import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import threading
import time
import urllib.request
import serial
import os
from collections import deque

# ─── CONFIG ───────────────────────────────────────────────────────────────────
ESP32_CAM_IP = "192.168.4.1"
STREAM_URL   = f"http://{ESP32_CAM_IP}/stream"

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE    = 115200

MODEL_PATH = "face.tflite"
MODEL_URL  = ("https://storage.googleapis.com/mediapipe-models/face_detector/"
              "blaze_face_short_range/float16/latest/blaze_face_short_range.tflite")

# Servo
PAN_CENTER, TILT_CENTER = 90, 90
PAN_MIN,  PAN_MAX       = 0, 180
TILT_MIN, TILT_MAX      = 0, 180

# PID (tunable)
PID_KP = 0.04
PID_KI = 0.001
PID_KD = 0.008

CENTER_TOL     = 25    # pixel dead-zone al centro
DETECT_SKIP    = 4    # esegue detection ogni N frame
NO_FACE_TIMEOUT = 3.0 # secondi senza volto prima di tornare al centro

# Smoothing posizione servo (media mobile)
SMOOTH_WIN = 5

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
        ser = None

def send_servo(p: int, t: int):
    if ser is None:
        return
    try:
        ser.write(f"{p},{t}\n".encode())
        ser.flush()
    except serial.SerialException:
        pass  # non blocca il loop principale

init_serial()

# ─── SHARED STATE ─────────────────────────────────────────────────────────────
latest_frame: np.ndarray | None = None
latest_faces: list               = []
frame_lock  = threading.Lock()
faces_lock  = threading.Lock()
running     = True

# ─── PID CONTROLLER ───────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, out_min=-10.0, out_max=10.0):
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

        self._integral  = np.clip(
            self._integral + error * dt, self.out_min / self.ki if self.ki else -1e9, self.out_max / self.ki if self.ki else 1e9
        )
        derivative = (error - self._prev_err) / dt
        self._prev_err = error

        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        return float(np.clip(out, self.out_min, self.out_max))

pid_pan  = PID(PID_KP, PID_KI, PID_KD, -8, 8)
pid_tilt = PID(PID_KP, PID_KI, PID_KD, -8, 8)

pan_buf  = deque([PAN_CENTER],  maxlen=SMOOTH_WIN)
tilt_buf = deque([TILT_CENTER], maxlen=SMOOTH_WIN)
pan      = PAN_CENTER
tilt     = TILT_CENTER

# ─── THREAD: STREAM READER (MJPEG manuale — zero buffer lag) ─────────────────
#
# cv2.VideoCapture accumula frame in un buffer interno che non si svuota mai
# abbastanza velocemente → ritardo crescente.
# Soluzione: leggere lo stream HTTP raw, estrarre i JPEG a mano e tenere
# SOLO l'ultimo frame disponibile, scartando tutto il resto.
#
def stream_reader():
    global latest_frame, running
    import urllib.request as urlreq

    BOUNDARY_TAG = b"--frame"
    SOI          = b"\xff\xd8"   # JPEG Start Of Image
    EOI          = b"\xff\xd9"   # JPEG End Of Image
    CHUNK        = 32768         # byte letti per iterazione (32 KB)

    while running:
        try:
            req  = urlreq.urlopen(STREAM_URL, timeout=5)
            print("Stream connesso.")
            buf  = b""

            while running:
                buf += req.read(CHUNK)

                # Cerca inizio e fine JPEG nel buffer grezzo
                start = buf.find(SOI)
                end   = buf.find(EOI, start + 2) + 2  # +2 per includere EOI

                if start == -1 or end < 2:
                    # JPEG incompleto — aspetta altri dati
                    if len(buf) > 200_000:
                        # Buffer troppo grande senza un JPEG valido → flush
                        buf = b""
                    continue

                jpg  = buf[start:end]
                # Scarta tutto ciò che precede il prossimo potenziale SOI
                buf  = buf[end:]

                # Decodifica
                arr   = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                frame = cv2.flip(frame, -1)

                with frame_lock:
                    latest_frame = frame

        except Exception as e:
            print(f"[WARN] Stream interrotto: {e} — riprovo...")
            time.sleep(2)

# ─── THREAD: FACE DETECTION ───────────────────────────────────────────────────
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
                bb = d.bounding_box
                score = d.categories[0].score if d.categories else 0.0
                faces.append((bb.origin_x, bb.origin_y, bb.width, bb.height, score))

        with faces_lock:
            latest_faces = faces

# ─── AVVIA THREAD ─────────────────────────────────────────────────────────────
threading.Thread(target=stream_reader,   daemon=True).start()
threading.Thread(target=detection_worker, daemon=True).start()

# ─── MAIN LOOP ────────────────────────────────────────────────────────────────
cv2.namedWindow("cam", cv2.WINDOW_NORMAL)

last_servo_time = 0.0
last_face_time  = time.monotonic()
SERVO_HZ = 30  # max frequenza invio comandi seriale

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

        # Prendi il volto più grande (più vicino)
        x, y, bw, bh, score = max(faces, key=lambda f: f[2] * f[3])

        cx = x + bw // 2
        cy = y + bh // 2

        err_x = cx - cx_frame
        err_y = cy - cy_frame

        # Dead-zone: non muovere se già centrato
        if abs(err_x) <= CENTER_TOL:
            err_x = 0
            pid_pan.reset()
        if abs(err_y) <= CENTER_TOL:
            err_y = 0
            pid_tilt.reset()

        # PID → delta angolo
        delta_pan  = -pid_pan.update(err_x)
        delta_tilt =  pid_tilt.update(err_y)

        pan_raw  = pan  + delta_pan
        tilt_raw = tilt + delta_tilt

        # Smoothing con media mobile
        pan_buf.append(np.clip(pan_raw,  PAN_MIN,  PAN_MAX))
        tilt_buf.append(np.clip(tilt_raw, TILT_MIN, TILT_MAX))

        pan  = int(np.mean(pan_buf))
        tilt = int(np.mean(tilt_buf))

        # Invia al servo con rate limiting
        now = time.monotonic()
        if now - last_servo_time >= 1.0 / SERVO_HZ:
            send_servo(pan, tilt)
            last_servo_time = now

        # HUD
        cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 220, 80), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 60, 255), -1)

        label = f"{score:.0%}  P:{pan}  T:{tilt}"
        cv2.putText(frame, label, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 80), 1, cv2.LINE_AA)

        # Linea dal centro al volto
        cv2.line(frame, (cx_frame, cy_frame), (cx, cy), (60, 60, 255), 1)

    else:
        pid_pan.reset()
        pid_tilt.reset()

        # Nessun volto da troppo tempo → torna al centro gradualmente
        if time.monotonic() - last_face_time >= NO_FACE_TIMEOUT:
            pan_buf.append(PAN_CENTER)
            tilt_buf.append(TILT_CENTER)
            pan  = int(np.mean(pan_buf))
            tilt = int(np.mean(tilt_buf))

            now = time.monotonic()
            if now - last_servo_time >= 1.0 / SERVO_HZ:
                send_servo(pan, tilt)
                last_servo_time = now

    # Mirino centrale
    cv2.line(frame, (cx_frame-20, cy_frame), (cx_frame+20, cy_frame), (200, 200, 200), 1)
    cv2.line(frame, (cx_frame, cy_frame-20), (cx_frame, cy_frame+20), (200, 200, 200), 1)

    cv2.imshow("cam", frame)

    if cv2.waitKey(1) == ord('q'):
        running = False
        break

if ser:
    ser.close()
cv2.destroyAllWindows()