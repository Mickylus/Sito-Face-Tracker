import cv2
import urllib.request
import numpy as np
import asyncio
import websockets
import json
import threading
import base64
import time

# ─── CONFIGURAZIONE ───────────────────────────────────────────
ESP32_IP   = "10.48.233.79"        # <-- IP del tuo ESP32
STREAM_URL = f"http://{ESP32_IP}/stream"

WS_HOST = "localhost"
WS_PORT = 8765                     # deve corrispondere a quello nella pagina web

FACE_CASCADE = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

SCALE_FACTOR  = 1.1
MIN_NEIGHBORS = 5
MIN_FACE_SIZE = (60, 60)
# ──────────────────────────────────────────────────────────────

# Stato condiviso tra i thread
connected_clients: set = set()
latest_result: dict = {"faces": []}
result_lock = threading.Lock()


# ─── WEBSOCKET SERVER ─────────────────────────────────────────

async def ws_handler(websocket):
    """Gestisce ogni client WebSocket connesso dalla pagina web."""
    connected_clients.add(websocket)
    addr = websocket.remote_address
    print(f"[WS] Client connesso: {addr[0]}:{addr[1]}")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get("type", "")

                # La pagina può inviare frame JPEG in base64 (modalità webcam browser)
                if msg_type == "frame":
                    img_b64 = data.get("image", "")
                    if img_b64:
                        img_bytes = base64.b64decode(img_b64)
                        np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if frame is not None:
                            result = detect_faces(frame)
                            await websocket.send(json.dumps(result))
            except Exception as e:
                print(f"[WS] Errore messaggio: {e}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"[WS] Client disconnesso: {addr[0]}:{addr[1]}")


async def broadcast(result: dict):
    """Invia il risultato a tutti i client connessi."""
    if not connected_clients:
        return
    msg = json.dumps(result)
    dead = set()
    for ws in connected_clients:
        try:
            await ws.send(msg)
        except Exception:
            dead.add(ws)
    connected_clients.difference_update(dead)


def run_ws_server():
    """Avvia il server WebSocket in un thread separato con il suo event loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    async def _serve():
        print(f"[WS] Server avviato su ws://{WS_HOST}:{WS_PORT}")
        async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
            await asyncio.Future()   # gira per sempre

    loop.run_until_complete(_serve())


# ─── RILEVAMENTO FACCE ────────────────────────────────────────

def detect_faces(frame) -> dict:
    """Rileva le facce nel frame e restituisce il risultato JSON per il sito."""
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray  = cv2.equalizeHist(gray)
    faces_raw = FACE_CASCADE.detectMultiScale(
        gray,
        scaleFactor=SCALE_FACTOR,
        minNeighbors=MIN_NEIGHBORS,
        minSize=MIN_FACE_SIZE,
        flags=cv2.CASCADE_SCALE_IMAGE,
    )

    faces_out = []
    h_frame, w_frame = frame.shape[:2]

    for (x, y, w, h) in (faces_raw if len(faces_raw) else []):
        cx = x + w // 2
        cy = y + h // 2

        # Confidenza simulata: dipende dalla dimensione relativa della faccia
        # (più grande = più vicina = più affidabile per Haar)
        area_ratio  = (w * h) / (w_frame * h_frame)
        confidence  = min(0.5 + area_ratio * 5, 0.99)

        faces_out.append({
            "name":       "Sconosciuto",   # sostituire con il tuo riconoscitore
            "recognized": False,
            "confidence": round(confidence, 2),
            "box": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
            "centroid":   {"cx": int(cx), "cy": int(cy)},
        })

    return {"faces": faces_out}


def draw_face(frame, face: dict):
    """Disegna bounding box e mirino sul frame OpenCV (finestra locale)."""
    b  = face["box"]
    cx = face["centroid"]["cx"]
    cy = face["centroid"]["cy"]
    x, y, w, h = b["x"], b["y"], b["w"], b["h"]

    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 6,  (0, 0, 255), -1)
    cv2.circle(frame, (cx, cy), 14, (0, 0, 255), 2)
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 1)

    label = f"X:{cx}  Y:{cy}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.rectangle(frame, (x, y - th - 10), (x + tw + 6, y), (0, 255, 0), -1)
    cv2.putText(frame, label, (x + 3, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)


# ─── LOOP PRINCIPALE ESP32 ────────────────────────────────────

def process_stream(ws_loop: asyncio.AbstractEventLoop):
    """Legge lo stream MJPEG dalla ESP32, rileva facce, invia al sito."""
    print(f"[ESP32] Connessione a {STREAM_URL} ...")
    stream = urllib.request.urlopen(STREAM_URL, timeout=10)
    buf = bytes()
    print("[ESP32] Stream connesso. Premi Q nella finestra OpenCV per uscire.")

    while True:
        buf += stream.read(4096)
        start = buf.find(b'\xff\xd8')
        end   = buf.find(b'\xff\xd9')
        if start == -1 or end == -1:
            continue

        jpg   = buf[start:end + 2]
        buf   = buf[end + 2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # ── Rilevamento ──
        result = detect_faces(frame)

        # ── Broadcast WebSocket (thread-safe) ──
        if connected_clients:
            asyncio.run_coroutine_threadsafe(broadcast(result), ws_loop)

        # ── Disegna sulla finestra locale ──
        h_frame, w_frame = frame.shape[:2]
        n = len(result["faces"])

        for face in result["faces"]:
            draw_face(frame, face)
            c = face["centroid"]
            print(f"  Faccia — centroide: X={c['cx']}, Y={c['cy']}  |  "
                  f"bbox: ({face['box']['x']},{face['box']['y']}) "
                  f"{face['box']['w']}x{face['box']['h']}px  |  "
                  f"confidenza: {face['confidence']:.0%}")

        status = f"{n} faccia/e rilevata/e" if n else "Nessuna faccia rilevata"
        color  = (0, 255, 0) if n else (0, 100, 255)
        ws_n   = len(connected_clients)

        cv2.rectangle(frame, (0, 0), (w_frame, 28), (0, 0, 0), -1)
        cv2.putText(frame,
                    f"{w_frame}x{h_frame}  |  {status}  |  WS client: {ws_n}",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

        cv2.imshow("ESP32-CAM — Face Detection  [Q per uscire]", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    print("[ESP32] Chiuso.")


# ─── ENTRY POINT ──────────────────────────────────────────────

if __name__ == "__main__":
    # 1) Avvia il server WebSocket in un thread dedicato
    ws_loop = asyncio.new_event_loop()

    ws_thread = threading.Thread(
        target=lambda: (
            asyncio.set_event_loop(ws_loop),
            ws_loop.run_until_complete(
                (lambda: (
                    print(f"[WS] Server avviato su ws://{WS_HOST}:{WS_PORT}"),
                    ws_loop.run_until_complete(
                        websockets.serve(ws_handler, WS_HOST, WS_PORT)
                    ),
                    ws_loop.run_forever()
                ))()
            )
        ),
        daemon=True,
    )

    # Versione più pulita del thread
    def start_ws():
        asyncio.set_event_loop(ws_loop)
        print(f"[WS] Server avviato su ws://{WS_HOST}:{WS_PORT}")

        async def _serve():
            async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
                await asyncio.Future()

        ws_loop.run_until_complete(_serve())

    ws_thread = threading.Thread(target=start_ws, daemon=True)
    ws_thread.start()
    time.sleep(0.5)   # lascia partire il server prima dello stream

    # 2) Loop principale nello stream ESP32 (thread principale)
    try:
        process_stream(ws_loop)
    except KeyboardInterrupt:
        print("\n[INFO] Interrotto dall'utente.")
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"[ERRORE] {e}")
        cv2.destroyAllWindows()