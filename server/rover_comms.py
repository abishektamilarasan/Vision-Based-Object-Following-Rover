import requests
import config
import threading
import queue
import time

class RoverComms:
    def __init__(self, base_url=config.ESP8266_MOTOR_URL):
        self.base_url = base_url
        self.last_left = 0
        self.last_right = 0
        
        self.session = requests.Session()
        # Add Keep-Alive and disable proxy lookups on localhost/LAN to prevent connection resets
        adapter = requests.adapters.HTTPAdapter(pool_connections=1, pool_maxsize=2, max_retries=1)
        self.session.mount("http://", adapter)
        self.session.trust_env = False
        
        self.command_queue = queue.Queue(maxsize=5) # Prevent queue buildup
        self.running = True
        
        self.is_connected = False
        self.last_error = None
        self.base_url = base_url.rstrip("/")
        if not self.base_url.startswith("http://") and not self.base_url.startswith("https://"):
            self.base_url = f"http://{self.base_url}"

        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()
        
    def send_command(self, left_speed, right_speed):
        """
        Queues motor speeds to be sent to the ESP8266. Avoids duplicates.
        """
        if left_speed == self.last_left and right_speed == self.last_right:
            return
            
        self.last_left = left_speed
        self.last_right = right_speed
        
        try:
            # If queue is full, drop the oldest command to prioritize newest
            if self.command_queue.full():
                try:
                    self.command_queue.get_nowait()
                except queue.Empty:
                    pass
            self.command_queue.put_nowait((left_speed, right_speed))
        except queue.Full:
            pass

    def _worker(self):
        while self.running:
            try:
                left, right = self.command_queue.get(timeout=0.1)
                url = f"{self.base_url}/control?left={left}&right={right}"
                try:
                    resp = self.session.get(url, timeout=0.8)
                    if resp.status_code == 200:
                        self.is_connected = True
                        print(f"[ESP32 Comms] -> Sent: left={left}, right={right} | ESP response: 200 OK")
                    else:
                        print(f"[ESP32 Comms] -> Sent: left={left}, right={right} | Error: HTTP {resp.status_code}")
                except requests.exceptions.RequestException as e:
                    self.is_connected = False
                    self.last_error = str(e)
                    print(f"[ESP32 Comms] -> FAILED connecting to {url}: {e}")
                self.command_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                pass
                
    def get_status(self):
        """
        Polls the ESP32 for sensor status using the persistent session.
        """
        try:
            url = f"{self.base_url}/status"
            response = self.session.get(url, timeout=0.8)
            if response.status_code == 200:
                self.is_connected = True
                return response.json()
        except requests.exceptions.RequestException as e:
            self.is_connected = False
            self.last_error = str(e)
        return None
        
    def stop(self):
        self.running = False
        self.session.close()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
