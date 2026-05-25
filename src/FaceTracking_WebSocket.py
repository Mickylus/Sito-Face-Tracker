import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import threading
import time
import serial
import os
import urllib.request
from collections import deque
import websocket

# ─── CONFIG ───────────────────────────────────────────────────────────────────
ESP32_IP  = "192.168.0.128"   # ← IP dell'ESP32
WS_URL    = f"ws://{ESP32_IP}/ws"

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE    = 115200

MODEL_PATH = "face.tflite"
MODEL_URL  = ("https://storage.googleapis.com/mediapipe-models/face_detector/"
              "blaze_face_short_range/float16/latest/blaze_face_short_range.tflite")

# Servo
PAN_CENTER,  TILT_CENTER = 90, 90
PAN_MIN,     PAN_MAX     = 0, 180
TILT_MIN,    TILT_MAX    = 0, 180

# ─── VELOCITÀ SERVO ──────────────────────────────────────────────────────────
# Unico parametro da toccare: 0.0 = fermo, 1.0 = massima velocità
SERVO_SPEED = 0.5

# I parametri sotto vengono calcolati automaticamente da SERVO_SPEED
# (non serve modificarli manualmente)
PID_KP = 0.01 + 0.07  * SERVO_SPEED   # reattività proporzionale
PID_KI = 0.0  + 0.002 * SERVO_SPEED   # correzione errore accumulato
PID_KD = 0.0  + 0.016 * SERVO_SPEED   # smorzamento oscillazioni
SMOOTH_WIN      = max(2, int(14 - 12 * SERVO_SPEED))  # 14=lento 2=veloce
CENTER_TOL      = int(50 - 30 * SERVO_SPEED)          # 50=lento 20=veloce

CENTER_TOL      = max(10, CENTER_TOL)
NO_FACE_TIMEOUT = 3.0
DETECT_SKIP     = 4
SERVO_HZ        = 30

# ─── MODEL ────────────────────────────────────────────────────────────────────
if not os.path.exists(MODEL_PATH):
    print("Download modello...")
    urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)

detector = vision.FaceDetector.create_from_options(
    vision.FaceDetectorOptions(
        base_options=python.BaseOptions(model_asset_path=MODEL_PATH),
        min_detection_confidence=0.55,
        min_suppression_threshold=0.3,
    )
)

# ─── SERIAL ───────────────────────────────────────────────────────────────────
ser = None
def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print(f"Seriale OK: {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"[WARN] Seriale non disponibile: {e}")

def send_servo(p: int, t: int):
    if ser is None:
        return
    try:
        ser.write(f"{p},{t}\n".encode())
        ser.flush()
    except serial.SerialException:
        pass

init_serial()

# ─── PID ──────────────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, out_min=-8.0, out_max=8.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = time.monotonic()

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def update(self, error: float) -> float:
        now = time.monotonic()
        dt  = max(now - self._prev_time, 1e-4)
        self._prev_time = now
        self._integral = float(np.clip(
            self._integral + error * dt,
            self.out_min / self.ki if self.ki else -1e9,
            self.out_max / self.ki if self.ki else  1e9
        ))
        derivative = (error - self._prev_err) / dt
        self._prev_err = error
        return float(np.clip(
            self.kp * error + self.ki * self._integral + self.kd * derivative,
            self.out_min, self.out_max
        ))

pid_pan  = PID(PID_KP, PID_KI, PID_KD)
pid_tilt = PID(PID_KP, PID_KI, PID_KD)

pan_buf  = deque([PAN_CENTER],  maxlen=SMOOTH_WIN)
tilt_buf = deque([TILT_CENTER], maxlen=SMOOTH_WIN)
pan      = PAN_CENTER
tilt     = TILT_CENTER

# ─── SHARED STATE ─────────────────────────────────────────────────────────────
latest_frame = None
latest_faces = []
frame_lock   = threading.Lock()
faces_lock   = threading.Lock()
running      = True

# ─── WEBSOCKET READER ─────────────────────────────────────────────────────────
# Ogni messaggio binario dal server è un JPEG completo → nessun parsing,
# nessun buffer, nessun lag. Il server pusha il frame non appena è pronto.
def on_message(ws_app, message):
    global latest_frame
    arr   = np.frombuffer(message, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return
    frame = cv2.flip(frame, -1)
    with frame_lock:
        latest_frame = frame

def on_error(ws_app, error):
    print(f"[WS] Errore: {error}")

def on_close(ws_app, code, msg):
    print("[WS] Connessione chiusa — riconnessione...")

def on_open(ws_app):
    print(f"[WS] Connesso a {WS_URL}")

def ws_reader():
    global running
    while running:
        try:
            app = websocket.WebSocketApp(
                WS_URL,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close,
            )
            # run_forever si blocca finché la connessione è aperta
            app.run_forever(ping_interval=10, ping_timeout=5)
        except Exception as e:
            print(f"[WS] Eccezione: {e}")
        if running:
            time.sleep(2)

# ─── FACE DETECTION ───────────────────────────────────────────────────────────
def detection_worker():
    global latest_faces, running
    local_id = 0

    while running:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.01)
                continue
            frame = latest_frame.copy()

        local_id += 1
        if local_id % DETECT_SKIP != 0:
            time.sleep(0.005)
            continue

        rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        res      = detector.detect(mp_image)

        faces = []
        if res.detections:
            for d in res.detections:
                bb    = d.bounding_box
                score = d.categories[0].score if d.categories else 0.0
                faces.append((bb.origin_x, bb.origin_y, bb.width, bb.height, score))

        with faces_lock:
            latest_faces = faces

# ─── AVVIA THREAD ─────────────────────────────────────────────────────────────
threading.Thread(target=ws_reader,        daemon=True).start()
threading.Thread(target=detection_worker, daemon=True).start()

# ─── MAIN LOOP ────────────────────────────────────────────────────────────────
cv2.namedWindow("cam", cv2.WINDOW_NORMAL)

last_servo_time = 0.0
last_face_time  = time.monotonic()

while True:

    with frame_lock:
        if latest_frame is None:
            time.sleep(0.01)
            continue
        frame = latest_frame.copy()

    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    with faces_lock:
        faces = list(latest_faces)

    if faces:
        last_face_time = time.monotonic()
        x, y, bw, bh, score = max(faces, key=lambda f: f[2] * f[3])

        cx = x + bw // 2
        cy = y + bh // 2

        err_x = cx - cx_frame
        err_y = cy - cy_frame

        if abs(err_x) <= CENTER_TOL:
            err_x = 0
            pid_pan.reset()
        if abs(err_y) <= CENTER_TOL:
            err_y = 0
            pid_tilt.reset()

        pan_buf.append(np.clip(pan  + (-pid_pan.update(err_x)),  PAN_MIN,  PAN_MAX))
        tilt_buf.append(np.clip(tilt + pid_tilt.update(err_y),   TILT_MIN, TILT_MAX))
        pan  = int(np.mean(pan_buf))
        tilt = int(np.mean(tilt_buf))

        now = time.monotonic()
        if now - last_servo_time >= 1.0 / SERVO_HZ:
            send_servo(pan, tilt)
            last_servo_time = now

        cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 220, 80), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 60, 255), -1)
        cv2.line(frame, (cx_frame, cy_frame), (cx, cy), (60, 60, 255), 1)
        cv2.putText(frame, f"{score:.0%}  P:{pan} T:{tilt}",
                    (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 80), 1, cv2.LINE_AA)

    else:
        pid_pan.reset()
        pid_tilt.reset()

        if time.monotonic() - last_face_time >= NO_FACE_TIMEOUT:
            pan_buf.append(PAN_CENTER)
            tilt_buf.append(TILT_CENTER)
            pan  = int(np.mean(pan_buf))
            tilt = int(np.mean(tilt_buf))
            now  = time.monotonic()
            if now - last_servo_time >= 1.0 / SERVO_HZ:
                send_servo(pan, tilt)
                last_servo_time = now

    # Mirino
    cv2.line(frame, (cx_frame-20, cy_frame), (cx_frame+20, cy_frame), (200, 200, 200), 1)
    cv2.line(frame, (cx_frame, cy_frame-20), (cx_frame, cy_frame+20), (200, 200, 200), 1)

    cv2.imshow("cam", frame)
    if cv2.waitKey(1) == ord('q'):
        running = False
        break

if ser:
    ser.close()
cv2.destroyAllWindows()