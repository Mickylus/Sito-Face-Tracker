import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import urllib.request
import numpy as np
import threading
import queue
import time
import os
import serial
from collections import deque

# ───────────────── CONFIG ─────────────────
ESP32_CAM_IP = "10.103.240.79"
STREAM_URL = f"http://{ESP32_CAM_IP}/stream"

SERIAL_PORT = "/dev/ttyUSB0"   # Linux
# SERIAL_PORT = "COM5"         # Windows

BAUDRATE = 115200

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "face_detector/blaze_face_short_range/float16/latest/"
    "blaze_face_short_range.tflite"
)

MODEL_PATH = "blaze_face_short_range.tflite"

MIN_CONFIDENCE = 0.6
DETECT_EVERY_N = 2
FRAME_QUEUE_SIZE = 2

CENTER_TOLERANCE = 35
SERVO_STEP = 2

frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
result_queue = queue.Queue(maxsize=2)
stop_event = threading.Event()

pan_angle = 90
tilt_angle = 90

serial_conn = None


# ───────────────── MODEL ─────────────────
def download_model():
    if not os.path.exists(MODEL_PATH):
        print("Download modello...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def make_detector():
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)

    options = vision.FaceDetectorOptions(
        base_options=base_options,
        min_detection_confidence=MIN_CONFIDENCE,
    )

    return vision.FaceDetector.create_from_options(options)


# ───────────────── SERIAL ─────────────────
def init_serial():
    global serial_conn

    serial_conn = serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        timeout=1
    )

    time.sleep(2)

    print(f"[SERIAL] Connesso a {SERIAL_PORT}")


def send_servo_angles(pan, tilt):
    global serial_conn

    if serial_conn is None:
        return

    msg = f"{pan},{tilt}\n"

    try:
        serial_conn.write(msg.encode())
    except Exception as e:
        print("[SERIAL ERROR]", e)


# ───────────────── MJPEG ─────────────────
def mjpeg_reader():
    while not stop_event.is_set():

        try:
            print(f"[STREAM] Connessione a {STREAM_URL}")

            stream = urllib.request.urlopen(STREAM_URL, timeout=10)

            buf = bytes()

            print("[STREAM] Connesso")

            while not stop_event.is_set():

                chunk = stream.read(8192)

                if not chunk:
                    break

                buf += chunk

                while True:

                    start = buf.find(b'\xff\xd8')
                    end = buf.find(b'\xff\xd9')

                    if start == -1 or end == -1 or end < start:
                        break

                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]

                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )

                    if frame is None:
                        continue

                    # 🔥 INVERSIONE CAMERA (MODIFICA FATTA QUI)
                    frame = cv2.flip(frame, -1)

                    if frame_queue.full():
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            pass

                    frame_queue.put(frame)

        except Exception as e:
            print("[STREAM ERROR]", e)
            time.sleep(1)


# ───────────────── DETECTOR ─────────────────
def face_detector_thread():

    detector = make_detector()

    frame_counter = 0
    last_faces = []

    while not stop_event.is_set():

        try:
            frame = frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        frame_counter += 1

        if frame_counter % DETECT_EVERY_N == 0:

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb
            )

            results = detector.detect(mp_image)

            last_faces = []

            if results.detections:

                for det in results.detections:

                    bb = det.bounding_box

                    score = det.categories[0].score

                    last_faces.append((
                        bb.origin_x,
                        bb.origin_y,
                        bb.width,
                        bb.height,
                        score
                    ))

        h, w = frame.shape[:2]

        result = (frame.copy(), list(last_faces), (w, h))

        if result_queue.full():
            try:
                result_queue.get_nowait()
            except queue.Empty:
                pass

        result_queue.put(result)


# ───────────────── TRACKING ─────────────────
def track_face(cx, cy, frame_w, frame_h):

    global pan_angle
    global tilt_angle

    center_x = frame_w // 2
    center_y = frame_h // 2

    error_x = cx - center_x
    error_y = cy - center_y

    if abs(error_x) > CENTER_TOLERANCE:
        if error_x > 0:
            pan_angle -= SERVO_STEP
        else:
            pan_angle += SERVO_STEP

    if abs(error_y) > CENTER_TOLERANCE:
        if error_y > 0:
            tilt_angle += SERVO_STEP
        else:
            tilt_angle -= SERVO_STEP

    pan_angle = max(0, min(180, pan_angle))
    tilt_angle = max(0, min(180, tilt_angle))

    send_servo_angles(pan_angle, tilt_angle)


# ───────────────── DRAW ─────────────────
def draw_face(frame, x, y, bw, bh, score):

    cx = x + bw // 2
    cy = y + bh // 2

    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

    txt = f"{score:.0%}"

    cv2.putText(
        frame,
        txt,
        (x, y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )

    return cx, cy


# ───────────────── DISPLAY ─────────────────
def display_loop():

    fps_buf = deque(maxlen=30)
    last_time = time.time()

    cv2.namedWindow("Face Tracking", cv2.WINDOW_NORMAL)

    while not stop_event.is_set():

        try:
            frame, faces, (w, h) = result_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        now = time.time()
        fps_buf.append(1.0 / max(now - last_time, 1e-6))
        last_time = now
        fps = sum(fps_buf) / len(fps_buf)

        center_x = w // 2
        center_y = h // 2

        cv2.line(frame, (center_x - 30, center_y),
                 (center_x + 30, center_y), (255, 255, 255), 1)

        cv2.line(frame, (center_x, center_y - 30),
                 (center_x, center_y + 30), (255, 255, 255), 1)

        if faces:

            biggest = max(faces, key=lambda f: f[2] * f[3])

            x, y, bw, bh, score = biggest

            cx, cy = draw_face(frame, x, y, bw, bh, score)

            track_face(cx, cy, w, h)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.putText(frame,
                    f"PAN:{pan_angle} TILT:{tilt_angle}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2)

        cv2.imshow("Face Tracking", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cv2.destroyAllWindows()


# ───────────────── MAIN ─────────────────
def main():

    download_model()
    init_serial()

    threads = [
        threading.Thread(target=mjpeg_reader, daemon=True),
        threading.Thread(target=face_detector_thread, daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        display_loop()
    except KeyboardInterrupt:
        stop_event.set()

    for t in threads:
        t.join(timeout=2)


if __name__ == "__main__":
    main()