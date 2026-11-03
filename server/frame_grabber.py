
import cv2
import threading
import time
import requests
import numpy as np
import config


def _get_camera_url():
    """Returns the active camera stream URL based on config.CAMERA_SOURCE."""
    source = getattr(config, "CAMERA_SOURCE", "esp32").lower()
    if source == "phone":
        return getattr(config, "PHONE_CAM_URL", config.ESP32_CAM_URL)
    return config.ESP32_CAM_URL


class FrameGrabber:
    def __init__(self, stream_url=None):
        self.stream_url = stream_url or _get_camera_url()
        self.latest_frame = None
        self.running = False
        self.lock = threading.Lock()
        self.thread = None
        self.response = None
        self._restart_flag = False  # Set to True to trigger a reconnect with new URL

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def switch_source(self, source):
        """
        Hot-swap camera source at runtime.
        source: "esp32" or "phone"
        """
        config.CAMERA_SOURCE = source
        new_url = _get_camera_url()

        if source == "phone":
            config.FLIP_HORIZONTAL = False
        else:
            config.FLIP_HORIZONTAL = True

        print(f"[FrameGrabber] Switching camera to: {source} ({new_url})")
        self.stream_url = new_url
        self._restart_flag = True  # Signal the read loop to break and reconnect

    def _update(self):
        import urllib.request

        # Ignore system proxies for local network IP
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

        while self.running:
            self._restart_flag = False
            stream = None
            try:
                print(f"Connecting to camera stream at {self.stream_url}...")
                req = urllib.request.Request(self.stream_url, headers={"User-Agent": "Mozilla/5.0"})
                # 3.0 second timeout to detect dead TCP streams immediately
                stream = opener.open(req, timeout=3.0)
                print("Camera stream connected.")

                buffer = b""
                last_frame_received_time = time.time()

                while self.running and not self._restart_flag:
                    # Watchdog: if stream stalled for > 4.0s without a valid frame, force reconnect
                    if time.time() - last_frame_received_time > 4.0:
                        print("Camera stream stalled (no frames decoded in 4s), reconnecting...")
                        break

                    try:
                        chunk = stream.read(4096)
                        if not chunk:
                            print("Camera stream ended.")
                            break
                        buffer += chunk
                    except Exception:
                        break

                    # Keep buffer compact (never let old chunks backlog)
                    if len(buffer) > 40960:
                        last_soi = buffer.rfind(b"\xff\xd8")
                        if last_soi != -1:
                            buffer = buffer[last_soi:]
                        else:
                            buffer = buffer[-8192:]

                    # Use rfind to jump directly to the freshest completed frame and discard old backlog
                    last_b = buffer.rfind(b"\xff\xd9")
                    if last_b != -1:
                        last_a = buffer[:last_b].rfind(b"\xff\xd8")
                        if last_a != -1 and last_a < last_b:
                            jpg = buffer[last_a:last_b + 2]
                            buffer = buffer[last_b + 2:]

                            frame = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8),
                                cv2.IMREAD_COLOR
                            )

                            if frame is not None and frame.size > 0:
                                last_frame_received_time = time.time()
                                frame = cv2.resize(
                                    frame,
                                    (config.FRAME_WIDTH, config.FRAME_HEIGHT)
                                )

                                if getattr(config, "FLIP_HORIZONTAL", False):
                                    frame = cv2.flip(frame, 1)

                                with self.lock:
                                    self.latest_frame = frame
                    elif len(buffer) > 300000:
                        buffer = b""

            except Exception as error:
                print(f"Camera connection error: {error}")

            finally:
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass

            if self.running:
                if self._restart_flag:
                    print("Camera source changed, reconnecting immediately...")
                    self._restart_flag = False
                else:
                    print("Retrying camera connection in 1 second...")
                    time.sleep(1)

    def read(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()

            return None

    def stop(self):
        self.running = False

        if self.response:
            self.response.close()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3)