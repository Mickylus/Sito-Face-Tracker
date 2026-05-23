# Serial video viewer
# Usage: ./myenv/bin/python src/face_serial.py
import serial
import struct
import time
import numpy as np
import cv2
import threading
import queue

PORT = '/dev/ttyUSB0'
BAUD = 2000000

ser = serial.Serial(PORT, BAUD, timeout=0.1)

def read_exact_serial(n):
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise IOError('Timeout')
        buf += chunk
    return buf

def reader_thread_fn(frame_q, stop_event):
    # legge continuamente dalla seriale e mette i frame in coda
    sync_buf = b''
    try:
        while not stop_event.is_set():
            # find magic
            c = ser.read(1)
            if not c:
                continue
            sync_buf += c
            if len(sync_buf) > 5:
                sync_buf = sync_buf[-5:]
            if sync_buf != b'FRAME':
                continue

            # read header
            raw_len = read_exact_serial(4)
            length = struct.unpack('<I', raw_len)[0]
            raw_ts = read_exact_serial(8)
            ts = struct.unpack('<Q', raw_ts)[0]
            data = read_exact_serial(length)

            # push to queue, keep only latest if full
            try:
                frame_q.put_nowait((ts, data))
            except queue.Full:
                try:
                    _ = frame_q.get_nowait()  # drop oldest
                except Exception:
                    pass
                try:
                    frame_q.put_nowait((ts, data))
                except Exception:
                    pass
    except Exception as e:
        print('Reader thread error:', e)

def main():
    frame_q = queue.Queue(maxsize=4)
    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread_fn, args=(frame_q, stop_event), daemon=True)
    t.start()

    cv2.namedWindow('Serial Stream', cv2.WINDOW_NORMAL)
    fps = 0.0
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            try:
                ts, data = frame_q.get(timeout=1.0)
            except queue.Empty:
                continue

            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                print('Failed to decode JPEG')
                continue

            frame_count += 1
            now = time.time()
            elapsed = now - start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start_time = now

            cv2.putText(img, f'FPS: {fps:.1f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(img, f'TS: {ts}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)

            cv2.imshow('Serial Stream', img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t.join(timeout=1.0)
        ser.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()