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
import asyncio
import websockets
import json
import base64
from collections import deque

# ─── CONFIGURAZIONE ───────────────────────────────────────────
ESP32_IP   = "10.48.233.79"
STREAM_URL = f"http://{ESP32_IP}/stream"

WS_HOST = "localhost"   # ascolta su tutte le interfacce
WS_PORT = 8765         # deve corrispondere all'HTML

MODEL_URL  = (
    "https://storage.googleapis.com/mediapipe-models/"
    "face_detector/blaze_face_short_range/float16/latest/"
    "blaze_face_short_range.tflite"
)
MODEL_PATH = "blaze_face_short_range.tflite"

MIN_CONFIDENCE  = 0.6
DETECT_EVERY_N  = 2
FRAME_QUEUE_SIZE = 2

# Qualità JPEG del frame spedito al browser (0–100)
STREAM_JPEG_QUALITY = 60
# ──────────────────────────────────────────────────────────────

frame_queue  = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
result_queue = queue.Queue(maxsize=2)
stop_event   = threading.Event()

connected_clients: set = set()
clients_lock = threading.Lock()
ws_loop: asyncio.AbstractEventLoop | None = None


# ─── DOWNLOAD MODELLO ─────────────────────────────────────────
def download_model():
    if not os.path.exists(MODEL_PATH):
        print("Download modello BlazeFace...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Modello scaricato.")


def make_detector() -> vision.FaceDetector:
    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
    options = vision.FaceDetectorOptions(
        base_options=base_options,
        min_detection_confidence=MIN_CONFIDENCE,
    )
    return vision.FaceDetector.create_from_options(options)


# ─── WEBSOCKET SERVER ─────────────────────────────────────────
async def ws_handler(websocket):
    with clients_lock:
        connected_clients.add(websocket)
    addr = websocket.remote_address
    print(f"[WS] Client connesso: {addr[0]}:{addr[1]}  "
          f"(totale: {len(connected_clients)})")
    try:
        async for _ in websocket:
            pass   # il browser non manda nulla, solo ascolta
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        with clients_lock:
            connected_clients.discard(websocket)
        print(f"[WS] Client disconnesso: {addr[0]}:{addr[1]}")


async def broadcast_payload(payload: str):
    with clients_lock:
        targets = set(connected_clients)
    if not targets:
        return
    dead = set()
    for ws in targets:
        try:
            await ws.send(payload)
        except Exception:
            dead.add(ws)
    with clients_lock:
        connected_clients.difference_update(dead)


def broadcast_sync(payload: str):
    """Chiamata dai thread non-async per inviare ai client."""
    if ws_loop and connected_clients:
        asyncio.run_coroutine_threadsafe(
            broadcast_payload(payload), ws_loop
        )


def start_ws_server():
    global ws_loop
    ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ws_loop)

    async def _serve():
        print(f"[WS] Server avviato su ws://localhost:{WS_PORT}")
        async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
            await asyncio.Future()

    ws_loop.run_until_complete(_serve())


# ─── THREAD 1: lettura stream MJPEG ───────────────────────────
def mjpeg_reader():
    while not stop_event.is_set():
        try:
            print(f"[reader] Connessione a {STREAM_URL} ...")
            stream = urllib.request.urlopen(STREAM_URL, timeout=10)
            buf = bytes()
            print("[reader] Stream connesso.")

            while not stop_event.is_set():
                chunk = stream.read(8192)
                if not chunk:
                    break
                buf += chunk

                while True:
                    start = buf.find(b'\xff\xd8')
                    end   = buf.find(b'\xff\xd9')
                    if start == -1 or end == -1 or end < start:
                        break

                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]

                    frame = cv2.imdecode(
                        np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is None:
                        continue

                    if frame_queue.full():
                        try:
                            frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    frame_queue.put(frame)

        except Exception as e:
            if not stop_event.is_set():
                print(f"[reader] Errore: {e} — riconnessione tra 1 s...")
                time.sleep(1)


# ─── THREAD 2: face detection ─────────────────────────────────
def face_detector_thread():
    detector      = make_detector()
    frame_counter = 0
    last_faces    = []

    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        frame_counter += 1

        if frame_counter % DETECT_EVERY_N == 0:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            results  = detector.detect(mp_image)

            last_faces = []
            if results.detections:
                for det in results.detections:
                    bb    = det.bounding_box
                    score = det.categories[0].score if det.categories else 0.0
                    last_faces.append((bb.origin_x, bb.origin_y,
                                       bb.width, bb.height, score))

        h, w = frame.shape[:2]
        result = (frame.copy(), list(last_faces), (w, h))

        if result_queue.full():
            try:
                result_queue.get_nowait()
            except queue.Empty:
                pass
        result_queue.put(result)


# ─── DISEGNO ──────────────────────────────────────────────────
def draw_face_cv(frame, x, y, bw, bh, score):
    cx = x + bw // 2
    cy = y + bh // 2
    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 6,  (0, 0, 255), -1)
    cv2.circle(frame, (cx, cy), 14, (0, 0, 255),  2)
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 1)
    label = f"X:{cx}  Y:{cy}  ({score:.0%})"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.rectangle(frame, (x, y - th - 10), (x + tw + 6, y), (0, 255, 0), -1)
    cv2.putText(frame, label, (x + 3, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return cx, cy


# ─── THREAD 3: display + broadcast WebSocket ──────────────────
def display_loop():
    fps_buf   = deque(maxlen=30)
    last_time = time.time()

    cv2.namedWindow("ESP32-CAM — BlazeFace  [Q per uscire]", cv2.WINDOW_NORMAL)
    cv2.startWindowThread()

    while not stop_event.is_set():
        try:
            frame, faces, (w_frame, h_frame) = result_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        now = time.time()
        fps_buf.append(1.0 / max(now - last_time, 1e-6))
        last_time = now
        fps = sum(fps_buf) / len(fps_buf)

        # Costruisce lista facce per il JSON
        faces_json = []
        for (x, y, bw, bh, score) in faces:
            cx, cy = draw_face_cv(frame, x, y, bw, bh, score)
            faces_json.append({
                "name":       "Sconosciuto",
                "recognized": False,
                "confidence": round(float(score), 2),
                "box":        {"x": int(x), "y": int(y),
                               "w": int(bw), "h": int(bh)},
                "centroid":   {"cx": int(cx), "cy": int(cy)},
            })
            print(f"  Faccia — centroide: X={cx}, Y={cy}  "
                  f"|  bbox: ({x},{y}) {bw}x{bh}px  |  conf: {score:.0%}")

        # HUD locale
        status = f"{len(faces)} faccia/e" if faces else "Nessuna faccia"
        color  = (0, 255, 0) if faces else (0, 100, 255)
        ws_n   = len(connected_clients)
        cv2.rectangle(frame, (0, 0), (w_frame, 28), (0, 0, 0), -1)
        cv2.putText(
            frame,
            f"{w_frame}x{h_frame}  |  FPS:{fps:.1f}  |  {status}  |  WS:{ws_n}",
            (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA,
        )

        cv2.imshow("ESP32-CAM — BlazeFace  [Q per uscire]", frame)

        # ── Broadcast al browser: frame JPEG + metadata facce ──
        if connected_clients:
            ok, buf = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY]
            )
            if ok:
                b64 = base64.b64encode(buf.tobytes()).decode("ascii")
                payload = json.dumps({
                    "frame": b64,
                    "faces": faces_json,
                    "fps":   round(fps, 1),
                    "res":   f"{w_frame}×{h_frame}",
                })
                broadcast_sync(payload)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
            break

    cv2.destroyAllWindows()
    print("Chiuso.")


# ─── ENTRY POINT ──────────────────────────────────────────────
def main():
    download_model()

    # WebSocket server in thread dedicato
    ws_thread = threading.Thread(target=start_ws_server, daemon=True, name="ws-server")
    ws_thread.start()
    time.sleep(0.5)

    threads = [
        threading.Thread(target=mjpeg_reader,         daemon=True, name="reader"),
        threading.Thread(target=face_detector_thread, daemon=True, name="detector"),
    ]
    for t in threads:
        t.start()

    try:
        display_loop()
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente.")
        stop_event.set()

    for t in threads:
        t.join(timeout=2)


if __name__ == "__main__":
    main()
