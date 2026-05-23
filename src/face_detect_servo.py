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
ESP32_CAM_IP = "192.168.4.1"
# 192.168.4.1
# 10.103.240.79
STREAM_URL   = f"http://{ESP32_CAM_IP}/stream"

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE    = 115200

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "face_detector/blaze_face_short_range/float16/latest/"
    "blaze_face_short_range.tflite"
)
MODEL_PATH = "blaze_face_short_range.tflite"

MIN_CONFIDENCE   = 0.6
DETECT_EVERY_N   = 4       # rileva 1 frame su 4
CENTER_TOLERANCE = 35
SERVO_STEP       = 1
SEND_INTERVAL    = 0.3   # max 20 comandi servo/sec

# ───────────────── STATE ─────────────────
stop_event  = threading.Event()
pan_angle   = 90
tilt_angle  = 90

serial_lock     = threading.Lock()
serial_conn     = None
_last_send_time = 0

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
    try:
        s = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        s.dtr = False
        s.rts = False
        time.sleep(2)
        s.reset_input_buffer()
        s.reset_output_buffer()
        with serial_lock:
            serial_conn = s
        print(f"[SERIAL] Connesso a {SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"[SERIAL] Init fallita: {e}")
        return False

def reconnect_serial():
    global serial_conn
    with serial_lock:
        if serial_conn:
            try:
                serial_conn.close()
            except:
                pass
        serial_conn = None
    time.sleep(2)
    return init_serial()

def send_servo_angles(pan, tilt):
    global serial_conn
    with serial_lock:
        if serial_conn is None or not serial_conn.is_open:
            return
        try:
            serial_conn.write(f"{int(pan)},{int(tilt)}\n".encode("ascii"))
            serial_conn.flush()
        except serial.SerialTimeoutException:
            pass
        except Exception as e:
            print(f"[SERIAL ERROR] {e}")
            try:
                serial_conn.close()
            except:
                pass
            serial_conn = None

def serial_watchdog():
   if not conn_ok:
    if serial_conn is None:
        reconnect_serial()

# ───────────────── TRACKING ─────────────────
def track_face(cx, cy, frame_w, frame_h):
    global pan_angle, tilt_angle, _last_send_time

    error_x = cx - frame_w // 2
    error_y = cy - frame_h // 2
    changed = False

    if abs(error_x) > CENTER_TOLERANCE:
        pan_angle += -SERVO_STEP if error_x > 0 else SERVO_STEP
        changed = True
    if abs(error_y) > CENTER_TOLERANCE:
        tilt_angle += SERVO_STEP if error_y > 0 else -SERVO_STEP
        changed = True

    pan_angle  = max(0, min(180, pan_angle))
    tilt_angle = max(0, min(180, tilt_angle))

    now = time.time()
    if changed and (now - _last_send_time) >= SEND_INTERVAL:
        send_servo_angles(pan_angle, tilt_angle)
        _last_send_time = now

# ───────────────── DRAW ─────────────────
def draw_face(frame, x, y, bw, bh, score):
    cx = x + bw // 2
    cy = y + bh // 2
    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)
    cv2.putText(frame, f"{score:.0%}", (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return cx, cy

# ───────────────── MAIN LOOP (tutto in un thread) ─────────────────
# ───────────────── MAIN LOOP (tutto in un thread) ─────────────────
def main_loop(detector):
    fps_buf       = deque(maxlen=30)
    last_time     = time.time()
    frame_counter = 0
    last_faces    = []

    TARGET_FPS = 30
    FRAME_TIME = 1.0 / TARGET_FPS
    last_frame_time = time.time()

    cv2.namedWindow("Face Tracking", cv2.WINDOW_NORMAL)

    while not stop_event.is_set():
        try:
            print(f"[STREAM] Connessione a {STREAM_URL}")
            stream = urllib.request.urlopen(STREAM_URL, timeout=10)
            print("[STREAM] Connesso")
            buf = bytearray()

            while not stop_event.is_set():

                # 🔽 FPS LIMIT (MODIFICA PRINCIPALE)
                now = time.time()
                if now - last_frame_time < FRAME_TIME:
                    time.sleep(FRAME_TIME - (now - last_frame_time))
                    continue
                last_frame_time = time.time()

                chunk = stream.read(4096)
                if not chunk:
                    break

                buf.extend(chunk)

                frames_decoded = []
                while True:
                    start = buf.find(b'\xff\xd8')
                    end   = buf.find(b'\xff\xd9', start)
                    if start == -1 or end == -1:
                        break
                    jpg = bytes(buf[start:end + 2])
                    del buf[:end + 2]
                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        frames_decoded.append(cv2.flip(frame, -1))

                if not frames_decoded:
                    continue

                frame = frames_decoded[-1]
                h, w  = frame.shape[:2]

                frame_counter += 1
                if frame_counter % DETECT_EVERY_N == 0:
                    rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    results  = detector.detect(mp_image)
                    last_faces = []
                    if results.detections:
                        for det in results.detections:
                            bb    = det.bounding_box
                            score = det.categories[0].score
                            last_faces.append((
                                bb.origin_x, bb.origin_y,
                                bb.width, bb.height, score
                            ))

                now = time.time()
                fps_buf.append(1.0 / max(now - last_time, 1e-6))
                last_time = now
                fps = sum(fps_buf) / len(fps_buf)

                cv2.line(frame, (w//2 - 30, h//2), (w//2 + 30, h//2), (255,255,255), 1)
                cv2.line(frame, (w//2, h//2 - 30), (w//2, h//2 + 30), (255,255,255), 1)

                if last_faces:
                    biggest = max(last_faces, key=lambda f: f[2] * f[3])
                    x, y, bw, bh, score = biggest
                    cx, cy = draw_face(frame, x, y, bw, bh, score)
                    track_face(cx, cy, w, h)

                with serial_lock:
                    serial_ok = serial_conn is not None and serial_conn.is_open
                serial_status = "SERIAL: OK" if serial_ok else "SERIAL: DISCONNESSA"
                serial_color  = (0, 255, 0) if serial_ok else (0, 0, 255)

                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, f"PAN:{pan_angle} TILT:{tilt_angle}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(frame, serial_status, (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, serial_color, 2)

                cv2.imshow("Face Tracking", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break

        except Exception as e:
            print(f"[LOOP ERROR] {e}")
            time.sleep(1)

    cv2.destroyAllWindows()

# ───────────────── MAIN ─────────────────
def main():
    download_model()
    init_serial()

    detector = make_detector()

    t_watchdog = threading.Thread(target=serial_watchdog, daemon=True)
    t_watchdog.start()

    try:
        main_loop(detector)
    except KeyboardInterrupt:
        stop_event.set()

    t_watchdog.join(timeout=2)

    with serial_lock:
        if serial_conn and serial_conn.is_open:
            serial_conn.close()

if __name__ == "__main__":
    main()