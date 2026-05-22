import cv2
import urllib.request
import numpy as np

# ─── CONFIGURAZIONE ───────────────────────────────────────────
ESP32_IP = "10.48.233.79"       # <-- metti l'IP del tuo ESP32
STREAM_URL = f"http://{ESP32_IP}/stream"

# Haar cascade incluso in OpenCV, nessun download necessario
FACE_CASCADE = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

# Parametri rilevamento (regolabili)
SCALE_FACTOR  = 1.1   # quanto ridurre l'immagine ad ogni scala (1.05–1.3)
MIN_NEIGHBORS = 5     # quanti vicini servono per confermare (più alto = meno falsi positivi)
MIN_FACE_SIZE = (60, 60)  # dimensione minima faccia in pixel
# ──────────────────────────────────────────────────────────────

def draw_face(frame, x, y, w, h):
    cx = x + w // 2
    cy = y + h // 2

    # Rettangolo faccia
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Centroide — cerchio + punto centrale
    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
    cv2.circle(frame, (cx, cy), 14, (0, 0, 255), 2)

    # Mirino a croce sul centroide
    cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 1)
    cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 1)

    # Label coordinate
    label = f"X:{cx}  Y:{cy}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.rectangle(frame, (x, y - th - 10), (x + tw + 6, y), (0, 255, 0), -1)
    cv2.putText(frame, label, (x + 3, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

    return cx, cy


def process_stream():
    print(f"Connessione a {STREAM_URL} ...")

    stream = urllib.request.urlopen(STREAM_URL, timeout=10)
    buf = bytes()

    print("Stream connesso. Premi Q per uscire.")

    while True:
        # Legge chunk per chunk fino a trovare un JPEG completo
        buf += stream.read(4096)

        start = buf.find(b'\xff\xd8')  # SOI JPEG
        end   = buf.find(b'\xff\xd9')  # EOI JPEG

        if start == -1 or end == -1:
            continue

        jpg = buf[start:end + 2]
        buf = buf[end + 2:]

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)  # migliora il contrasto per il rilevamento

        faces = FACE_CASCADE.detectMultiScale(
            gray,
            scaleFactor=SCALE_FACTOR,
            minNeighbors=MIN_NEIGHBORS,
            minSize=MIN_FACE_SIZE,
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        h_frame, w_frame = frame.shape[:2]

        if len(faces) == 0:
            status = "Nessuna faccia rilevata"
            color  = (0, 100, 255)
        else:
            status = f"{len(faces)} faccia/e rilevata/e"
            color  = (0, 255, 0)

            for (x, y, w, h) in faces:
                cx, cy = draw_face(frame, x, y, w, h)
                print(f"  Faccia — centroide: X={cx}, Y={cy}  |  bbox: ({x},{y}) {w}x{h}px")

        # HUD — risoluzione e stato
        cv2.rectangle(frame, (0, 0), (w_frame, 28), (0, 0, 0), -1)
        cv2.putText(frame, f"{w_frame}x{h_frame}  |  {status}",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        cv2.imshow("ESP32-CAM — Face Detection  [Q per uscire]", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    print("Chiuso.")


if __name__ == "__main__":
    try:
        process_stream()
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente.")
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"Errore: {e}")
        cv2.destroyAllWindows()
