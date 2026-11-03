import customtkinter as ctk
import cv2
from PIL import Image, ImageTk, ImageDraw, ImageFont
import threading
import time
import sys
import asyncio
import websockets
import json
import base64
import numpy as np
import logging

import config
from frame_grabber import FrameGrabber
from detector import Detector
from aruco_detector import ArucoDetector
from obstacle_avoidance import ObstacleAvoidance
from tracker import Tracker
from navigator import Navigator
from rover_comms import RoverComms
from identifier import CLIPIdentifier
from kalman_filter import ArucoKalmanFilter

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Main")

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

# ─── Colors ───
BG_DARK = "#1a1a2e"
BG_CARD = "#16213e"
BG_HEADER = "#0f3460"
ACCENT_BLUE = "#4361ee"
ACCENT_GREEN = "#06d6a0"
ACCENT_RED = "#ef476f"
ACCENT_ORANGE = "#ff9f1c"
ACCENT_PURPLE = "#7b2ff7"
TEXT_PRIMARY = "#e0e0e0"
TEXT_DIM = "#8d99ae"


class ObjectSelectionDialog(ctk.CTkToplevel):
    """Popup dialog showing detected objects so the user can pick one."""

    def __init__(self, parent, detections, frame):
        super().__init__(parent)
        self.title("Select Object to Follow")
        self.geometry("500x420")
        self.configure(fg_color=BG_DARK)
        self.resizable(False, False)
        self.grab_set()  # modal

        self.selected = None

        # Header
        header = ctk.CTkLabel(
            self, text="🎯  Tap an object to follow",
            font=("Segoe UI", 18, "bold"), text_color=TEXT_PRIMARY
        )
        header.pack(pady=(20, 10))

        hint = ctk.CTkLabel(
            self, text=f"{len(detections)} object(s) detected",
            font=("Segoe UI", 13), text_color=TEXT_DIM
        )
        hint.pack(pady=(0, 10))

        # Scrollable list of detections
        scroll = ctk.CTkScrollableFrame(self, fg_color=BG_CARD, corner_radius=12)
        scroll.pack(fill="both", expand=True, padx=20, pady=(0, 10))

        for i, det in enumerate(detections):
            row = ctk.CTkFrame(scroll, fg_color=BG_HEADER, corner_radius=10, height=60)
            row.pack(fill="x", pady=4, padx=4)
            row.pack_propagate(False)

            # Thumbnail
            x, y, w, h = det["bbox"]
            crop = frame[y:y+h, x:x+w]
            if crop.size > 0:
                crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                crop_pil = Image.fromarray(crop_rgb).resize((50, 50))
                crop_ctk = ctk.CTkImage(light_image=crop_pil, dark_image=crop_pil, size=(50, 50))
                thumb = ctk.CTkLabel(row, image=crop_ctk, text="")
                thumb.image = crop_ctk
                thumb.pack(side="left", padx=(10, 8), pady=5)

            # Show CLIP similarity if available
            sim_str = ""
            if 'clip_similarity' in det:
                sim_str = f"   CLIP: {det['clip_similarity']:.0%}"
            
            info_text = f"{det['class'].upper()}   •   {det['confidence']:.0%}{sim_str}"
            label = ctk.CTkLabel(
                row, text=info_text,
                font=("Segoe UI", 14, "bold"), text_color=TEXT_PRIMARY, anchor="w"
            )
            label.pack(side="left", fill="x", expand=True, padx=5)

            select_btn = ctk.CTkButton(
                row, text="Follow", width=80,
                fg_color=ACCENT_GREEN, hover_color="#05b88a",
                text_color="#000", font=("Segoe UI", 12, "bold"),
                corner_radius=8,
                command=lambda d=det: self._select(d)
            )
            select_btn.pack(side="right", padx=10, pady=5)

        # Cancel
        cancel_btn = ctk.CTkButton(
            self, text="Cancel", fg_color=BG_CARD, hover_color=BG_HEADER,
            text_color=TEXT_DIM, corner_radius=8, width=120,
            command=self.destroy
        )
        cancel_btn.pack(pady=(0, 15))

    def _select(self, det):
        self.selected = det
        self.destroy()


class RoverAppGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("🤖 Rover Control Station")
        self.geometry("1100x750")
        self.configure(fg_color=BG_DARK)

        # Core Modules
        self.grabber = FrameGrabber()
        self.detector = Detector()
        self.aruco_detector = ArucoDetector()
        self.obstacle_avoidance = ObstacleAvoidance()
        self.active_mode = getattr(config, "ACTIVE_MODE", "bag")
        self.identifier = CLIPIdentifier()
        self.aruco_kalman = ArucoKalmanFilter()
        self.tracker = Tracker(identifier=self.identifier)
        self.navigator = Navigator()
        self.comms = RoverComms()

        self.is_following = False
        self.running = True

        # Autonomous Wander Mode (triggered 60s after tracking stopped/lost)
        self.stopped_tracking_time = time.time()
        self.is_wandering = False
        self.wander_turn_until = 0
        self.wander_turn_dir = 1  # 1 for right, -1 for left
        self.manual_gui_driving = False

        self.connected_clients = set()
        self.ws_loop = None

        
        # Reference photos received from Android
        self.pending_references = []

        self.setup_ui()

        # Start ML Thread
        self.ml_thread = threading.Thread(target=self.ml_loop, daemon=True)
        self.ml_thread.start()

        # Start Telemetry Thread (polls ESP8266 asynchronously so video never freezes)
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        # Start WebSocket Server
        self.ws_thread = threading.Thread(target=self.start_ws_server, daemon=True)
        self.ws_thread.start()

    def telemetry_loop(self):
        while self.running:
            try:
                status = self.comms.get_status()
                if status:
                    lb = status.get("left_blocked", False)
                    rb = status.get("right_blocked", False)
                    self.obstacle_avoidance.update_sensor_state(lb, rb)
                    self.after(0, lambda left_b=lb, right_b=rb: (
                        self.esp_indicator.configure(
                            text="⚡ ESP32: Online",
                            text_color=ACCENT_GREEN
                        ),
                        self.tele_ir_left.configure(
                            text=f"IR Left:      {'BLOCKED' if left_b else 'CLEAR'}",
                            text_color=ACCENT_RED if left_b else ACCENT_GREEN
                        ),
                        self.tele_ir_right.configure(
                            text=f"IR Right:     {'BLOCKED' if right_b else 'CLEAR'}",
                            text_color=ACCENT_RED if right_b else ACCENT_GREEN
                        ),
                    ))
                else:
                    self.after(0, lambda: self.esp_indicator.configure(
                        text="⚡ ESP32: Offline (Check IP/Power)",
                        text_color=ACCENT_RED
                    ))
            except Exception:
                self.after(0, lambda: self.esp_indicator.configure(
                    text="⚡ ESP32: Error",
                    text_color=ACCENT_RED
                ))
            time.sleep(0.35)

    # ─────────────────────────── UI ───────────────────────────

    def setup_ui(self):
        # Header bar
        header_bar = ctk.CTkFrame(self, fg_color=BG_HEADER, height=50, corner_radius=0)
        header_bar.pack(fill="x", side="top")
        header_bar.pack_propagate(False)

        ctk.CTkLabel(
            header_bar, text="🤖  ROVER CONTROL STATION",
            font=("Segoe UI", 16, "bold"), text_color="#ffffff"
        ).pack(side="left", padx=20)

        self.conn_indicator = ctk.CTkLabel(
            header_bar, text="📱 No Mobile", font=("Segoe UI", 12),
            text_color=TEXT_DIM
        )
        self.conn_indicator.pack(side="right", padx=(5, 20))

        self.esp_indicator = ctk.CTkLabel(
            header_bar, text="⚡ ESP32: Connecting...", font=("Segoe UI", 12, "bold"),
            text_color=ACCENT_ORANGE
        )
        self.esp_indicator.pack(side="right", padx=10)

        # Main content area
        content = ctk.CTkFrame(self, fg_color=BG_DARK)
        content.pack(fill="both", expand=True, padx=10, pady=10)

        # ── Right: Control Panel ──
        panel = ctk.CTkFrame(content, fg_color=BG_CARD, corner_radius=12, width=300)
        panel.pack(side="right", fill="y", padx=(8, 0))
        panel.pack_propagate(False) # Prevent shrinking

        # ── Left: Video Feed ──
        video_card = ctk.CTkFrame(content, fg_color=BG_CARD, corner_radius=12)
        video_card.pack(side="left", fill="both", expand=True, padx=(0, 8))

        video_header = ctk.CTkFrame(video_card, fg_color="transparent")
        video_header.pack(fill="x", padx=15, pady=(12, 5))
        ctk.CTkLabel(
            video_header, text="📷 LIVE FEED", font=("Segoe UI", 13, "bold"),
            text_color=TEXT_DIM
        ).pack(side="left")

        self.fps_label = ctk.CTkLabel(
            video_header, text="-- FPS", font=("Segoe UI", 12),
            text_color=ACCENT_GREEN
        )
        self.fps_label.pack(side="right")

        self.video_label = ctk.CTkLabel(
            video_card, text="⏳ Connecting to camera...",
            font=("Segoe UI", 14), text_color=TEXT_DIM
        )
        self.video_label.pack(expand=True, fill="both", padx=10, pady=(0, 10))
        self.video_label.bind("<Button-1>", self.on_video_click)

        # Status
        status_card = ctk.CTkFrame(panel, fg_color=BG_HEADER, corner_radius=10)
        status_card.pack(fill="x", padx=12, pady=(12, 8))

        ctk.CTkLabel(
            status_card, text="STATUS", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(8, 0))

        self.status_label = ctk.CTkLabel(
            status_card, text="IDLE",
            font=("Segoe UI", 22, "bold"), text_color=TEXT_PRIMARY
        )
        self.status_label.pack(pady=(2, 4))

        self.target_label = ctk.CTkLabel(
            status_card, text="No target selected",
            font=("Segoe UI", 11), text_color=TEXT_DIM
        )
        self.target_label.pack(pady=(0, 4))

        self.clip_label = ctk.CTkLabel(
            status_card, text="",
            font=("Segoe UI", 10), text_color=ACCENT_PURPLE
        )
        self.clip_label.pack(pady=(0, 10))

        # Action Buttons
        btn_frame = ctk.CTkFrame(panel, fg_color="transparent")
        btn_frame.pack(fill="x", padx=12, pady=4)

        # ── Camera Source Selector ──
        ctk.CTkLabel(
            btn_frame, text="📷 CAMERA SOURCE", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(0, 4))

        initial_cam = "📱 Phone Camera (IP Webcam)" if getattr(config, "CAMERA_SOURCE", "esp32") == "phone" else "🔌 ESP32-CAM"
        self.cam_selector = ctk.CTkOptionMenu(
            btn_frame,
            values=["📱 Phone Camera (IP Webcam)", "🔌 ESP32-CAM"],
            command=self.switch_camera_gui,
            font=("Segoe UI", 12),
            fg_color=BG_CARD,
            button_color=ACCENT_PURPLE,
            button_hover_color="#6a1fd6"
        )
        self.cam_selector.set(initial_cam)
        self.cam_selector.pack(fill="x", pady=(0, 10))

        # ── AI Architecture Selector ──
        ctk.CTkLabel(
            btn_frame, text="AI ARCHITECTURE", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(0, 4))
        
        mode_map = {"bag": "Custom Bag Model", "general": "General Model (Old)", "aruco": "ArUco Marker Follower"}
        initial_mode = mode_map.get(getattr(config, "ACTIVE_MODE", "bag"), "Custom Bag Model")

        self.model_selector = ctk.CTkOptionMenu(
            btn_frame,
            values=["Custom Bag Model", "General Model (Old)", "ArUco Marker Follower"],
            command=self.switch_model_gui,
            font=("Segoe UI", 12),
            fg_color=BG_CARD,
            button_color=ACCENT_BLUE,
            button_hover_color="#3651d4"
        )
        self.model_selector.set(initial_mode)
        self.model_selector.pack(fill="x", pady=(0, 15))

        self.start_btn = ctk.CTkButton(
            btn_frame, text="▶  Start Following",
            command=self.start_following,
            fg_color=ACCENT_GREEN, hover_color="#05b88a",
            text_color="#000", font=("Segoe UI", 13, "bold"),
            corner_radius=10, height=42
        )
        self.start_btn.pack(fill="x", pady=4)

        self.stop_btn = ctk.CTkButton(
            btn_frame, text="⏹  Stop",
            command=self.stop_following,
            fg_color=ACCENT_RED, hover_color="#d63a5e",
            text_color="#fff", font=("Segoe UI", 13, "bold"),
            corner_radius=10, height=42
        )
        self.stop_btn.pack(fill="x", pady=4)

        self.reselect_btn = ctk.CTkButton(
            btn_frame, text="🎯  Select Target",
            command=self.open_selection_popup,
            fg_color=ACCENT_BLUE, hover_color="#3651d4",
            text_color="#fff", font=("Segoe UI", 13, "bold"),
            corner_radius=10, height=42
        )
        self.reselect_btn.pack(fill="x", pady=4)

        self.clear_btn = ctk.CTkButton(
            btn_frame, text="✕  Clear Target",
            command=self.clear_target,
            fg_color=BG_HEADER, hover_color="#1a3a6b",
            text_color=TEXT_DIM, font=("Segoe UI", 12),
            corner_radius=10, height=36
        )
        self.clear_btn.pack(fill="x", pady=4)

        # ── Manual Drive D-Pad ──
        drive_card = ctk.CTkFrame(panel, fg_color=BG_HEADER, corner_radius=10)
        drive_card.pack(fill="x", padx=12, pady=(8, 8))

        ctk.CTkLabel(
            drive_card, text="🎮 MANUAL DRIVE (WASD / Arrows)", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(6, 4))

        dpad_grid = ctk.CTkFrame(drive_card, fg_color="transparent")
        dpad_grid.pack(pady=(0, 8))

        btn_fwd = ctk.CTkButton(
            dpad_grid, text="▲\n(W)", width=60, height=36, font=("Segoe UI", 11, "bold"),
            fg_color=ACCENT_BLUE, hover_color="#3651d4"
        )
        btn_fwd.grid(row=0, column=1, padx=3, pady=2)
        btn_fwd.bind("<ButtonPress-1>", lambda e: self.manual_press(config.MAX_SPEED, config.MAX_SPEED))
        btn_fwd.bind("<ButtonRelease-1>", lambda e: self.manual_release())

        btn_left = ctk.CTkButton(
            dpad_grid, text="◄ (A)", width=60, height=36, font=("Segoe UI", 11, "bold"),
            fg_color=BG_CARD, hover_color="#1f2d52"
        )
        btn_left.grid(row=1, column=0, padx=3, pady=2)
        btn_left.bind("<ButtonPress-1>", lambda e: self.manual_press(-config.MAX_SPEED, config.MAX_SPEED))
        btn_left.bind("<ButtonRelease-1>", lambda e: self.manual_release())

        btn_stop = ctk.CTkButton(
            dpad_grid, text="■\nSTOP", width=60, height=36, font=("Segoe UI", 10, "bold"),
            fg_color=ACCENT_RED, hover_color="#d63a5e", command=self.manual_release
        )
        btn_stop.grid(row=1, column=1, padx=3, pady=2)

        btn_right = ctk.CTkButton(
            dpad_grid, text="(D) ►", width=60, height=36, font=("Segoe UI", 11, "bold"),
            fg_color=BG_CARD, hover_color="#1f2d52"
        )
        btn_right.grid(row=1, column=2, padx=3, pady=2)
        btn_right.bind("<ButtonPress-1>", lambda e: self.manual_press(config.MAX_SPEED, -config.MAX_SPEED))
        btn_right.bind("<ButtonRelease-1>", lambda e: self.manual_release())

        btn_rev = ctk.CTkButton(
            dpad_grid, text="▼\n(S)", width=60, height=36, font=("Segoe UI", 11, "bold"),
            fg_color=BG_CARD, hover_color="#1f2d52"
        )
        btn_rev.grid(row=2, column=1, padx=3, pady=2)
        btn_rev.bind("<ButtonPress-1>", lambda e: self.manual_press(-config.MAX_SPEED, -config.MAX_SPEED))
        btn_rev.bind("<ButtonRelease-1>", lambda e: self.manual_release())

        # CLIP Info
        self.clip_card = ctk.CTkFrame(panel, fg_color=BG_HEADER, corner_radius=10)
        self.clip_card.pack(fill="x", padx=12, pady=(4, 8))

        ctk.CTkLabel(
            self.clip_card, text="🧠 CLIP IDENTIFIER", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(6, 2))

        self.clip_refs_label = ctk.CTkLabel(
            self.clip_card, text="References: 0",
            font=("Consolas", 11), text_color=TEXT_PRIMARY, anchor="w"
        )
        self.clip_refs_label.pack(fill="x", padx=15)

        self.clip_sim_label = ctk.CTkLabel(
            self.clip_card, text="Similarity:  --",
            font=("Consolas", 11), text_color=TEXT_PRIMARY, anchor="w"
        )
        self.clip_sim_label.pack(fill="x", padx=15, pady=(0, 6))

        # Telemetry
        tele_card = ctk.CTkFrame(panel, fg_color=BG_HEADER, corner_radius=10)
        tele_card.pack(fill="x", padx=12, pady=(4, 8))

        ctk.CTkLabel(
            tele_card, text="TELEMETRY", font=("Segoe UI", 10, "bold"),
            text_color=TEXT_DIM
        ).pack(pady=(6, 2))

        self.tele_left = ctk.CTkLabel(
            tele_card, text="Left Motor:   0",
            font=("Consolas", 11), text_color=TEXT_PRIMARY, anchor="w"
        )
        self.tele_left.pack(fill="x", padx=15)

        self.tele_right = ctk.CTkLabel(
            tele_card, text="Right Motor:  0",
            font=("Consolas", 11), text_color=TEXT_PRIMARY, anchor="w"
        )
        self.tele_right.pack(fill="x", padx=15)

        self.tele_ir_left = ctk.CTkLabel(
            tele_card, text="IR Left:      CLEAR",
            font=("Consolas", 11), text_color=ACCENT_GREEN, anchor="w"
        )
        self.tele_ir_left.pack(fill="x", padx=15)

        self.tele_ir_right = ctk.CTkLabel(
            tele_card, text="IR Right:     CLEAR",
            font=("Consolas", 11), text_color=ACCENT_GREEN, anchor="w"
        )
        self.tele_ir_right.pack(fill="x", padx=15, pady=(0, 6))

        # Bind keyboard controls for manual driving
        self.bind("<KeyPress>", self.on_key_press)
        self.bind("<KeyRelease>", self.on_key_release)
        self.pressed_keys = set()

    # ─────────────────── Manual Keyboard Driving ───────────────────

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in ("w", "up", "s", "down", "a", "left", "d", "right", "space"):
            self.pressed_keys.add(key)
            self._update_keyboard_driving()

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys:
            self.pressed_keys.discard(key)
            self._update_keyboard_driving()

    def _update_keyboard_driving(self):
        if self.is_following:
            return  # Auto-following takes precedence

        self.is_wandering = False
        self.stopped_tracking_time = time.time()

        if "space" in self.pressed_keys or not self.pressed_keys:
            self.comms.send_command(0, 0)
            self.tele_left.configure(text="Left Motor:   0")
            self.tele_right.configure(text="Right Motor:  0")
            return

        spd = getattr(config, "MAX_SPEED", 200)
        fwd = ("w" in self.pressed_keys) or ("up" in self.pressed_keys)
        back = ("s" in self.pressed_keys) or ("down" in self.pressed_keys)
        left = ("a" in self.pressed_keys) or ("left" in self.pressed_keys)
        right = ("d" in self.pressed_keys) or ("right" in self.pressed_keys)

        left_s, right_s = 0, 0
        if fwd:
            left_s, right_s = spd, spd
        elif back:
            left_s, right_s = -spd, -spd

        if left:
            if fwd or back:
                left_s = int(left_s * 0.4)
            else:
                left_s, right_s = -spd, spd
        elif right:
            if fwd or back:
                right_s = int(right_s * 0.4)
            else:
                left_s, right_s = spd, -spd

        self.comms.send_command(left_s, right_s)
        self.tele_left.configure(text=f"Left Motor:   {left_s}")
        self.tele_right.configure(text=f"Right Motor:  {right_s}")

    def manual_press(self, left_s, right_s):
        if not self.is_following:
            self.manual_gui_driving = True
            self.is_wandering = False
            self.stopped_tracking_time = time.time()
            self.comms.send_command(left_s, right_s)
            self.tele_left.configure(text=f"Left Motor:   {left_s}")
            self.tele_right.configure(text=f"Right Motor:  {right_s}")

    def manual_release(self):
        self.manual_gui_driving = False
        if not self.is_following:
            self.stopped_tracking_time = time.time()
            self.comms.send_command(0, 0)
            self.tele_left.configure(text="Left Motor:   0")
            self.tele_right.configure(text="Right Motor:  0")

    # ─────────────────── Object Selection ───────────────────

    def switch_camera_gui(self, choice):
        """Hot-swap camera source from GUI dropdown."""
        if "Phone" in choice:
            self.grabber.switch_source("phone")
            self.video_label.configure(text="📱 Switching to Phone Camera...")
            logger.info("GUI switching camera to: Phone (IP Webcam)")
        else:
            self.grabber.switch_source("esp32")
            self.video_label.configure(text="🔌 Switching to ESP32-CAM...")
            logger.info("GUI switching camera to: ESP32-CAM")

    def switch_model_gui(self, choice):
        if choice == "Custom Bag Model":
            self.active_mode = "bag"
            config.YOLO_MODEL = "../models/custom_bag.pt"
            config.YOLO_CONFIDENCE_THRESHOLD = 0.50
            self.detector = Detector(config.YOLO_MODEL)
            self.clip_card.pack(fill="x", padx=12, pady=(4, 8))
            logger.info(f"GUI switching architecture to: {choice} ({config.YOLO_MODEL})")
        elif choice == "ArUco Marker Follower":
            self.active_mode = "aruco"
            # In ArUco mode, hide CLIP card since ArUco uses marker IDs directly
            self.clip_card.pack_forget()
            logger.info("GUI switching architecture to: ArUco Marker Follower (OpenCV ArUco)")
        else:
            self.active_mode = "general"
            config.YOLO_MODEL = "../models/yolov8n.pt"
            config.YOLO_CONFIDENCE_THRESHOLD = 0.30
            self.detector = Detector(config.YOLO_MODEL)
            self.clip_card.pack(fill="x", padx=12, pady=(4, 8))
            logger.info(f"GUI switching architecture to: {choice} ({config.YOLO_MODEL})")

    def on_video_click(self, event):
        self.open_selection_popup()

    def open_selection_popup(self):
        frame = self.grabber.read()
        if frame is None:
            return

        if self.active_mode == "aruco":
            detections = self.aruco_detector.detect(frame)
        else:
            detections = self.detector.detect(frame)
        if not detections:
            self.status_label.configure(text="NO OBJECTS", text_color=ACCENT_ORANGE)
            self.after(2000, lambda: self.status_label.configure(
                text="IDLE", text_color=TEXT_PRIMARY
            ))
            return

        dialog = ObjectSelectionDialog(self, detections, frame)
        self.wait_window(dialog)

        if dialog.selected:
            self.tracker.init_tracker(frame, dialog.selected)
            self.start_following()
            self.target_label.configure(
                text=f"🎯 {dialog.selected['class'].upper()}"
            )
            if self.active_mode != "aruco":
                refs = len(self.identifier.reference_embeddings)
                self.clip_refs_label.configure(text=f"References: {refs}")
                self.clip_label.configure(text="CLIP reference registered ✓")
            else:
                self.clip_label.configure(text="")



    def handle_reference_photos(self, photos_b64):
        """
        Process reference photos sent from Android.
        photos_b64 is a list of base64-encoded JPEG images.
        """
        self.identifier.clear_references()
        
        for i, b64_str in enumerate(photos_b64):
            try:
                img_data = base64.b64decode(b64_str)
                np_arr = np.frombuffer(img_data, np.uint8)
                crop = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if crop is not None:
                    self.identifier.register_reference(crop)
                    logger.info(f"Registered Android reference photo {i+1}")
            except Exception as e:
                logger.error(f"Failed to process reference photo {i+1}: {e}")
        
        refs = len(self.identifier.reference_embeddings)
        self.after(0, lambda r=refs: (
            self.clip_refs_label.configure(text=f"References: {r}"),
            self.clip_label.configure(text=f"📸 {r} photos received from Android"),
            self.status_label.configure(text="MATCHING", text_color=ACCENT_PURPLE),
        ))
        
        # Now try to find a match in the current frame
        self.after(500, self._match_references)

    def _match_references(self, attempts=0):
        """Try to match the registered reference photos against current detections."""
        frame = self.grabber.read()
        if frame is None:
            if attempts < 10:
                self.after(200, lambda: self._match_references(attempts + 1))
            return
        
        if self.active_mode == "aruco":
            detections = self.aruco_detector.detect(frame)
        else:
            detections = self.detector.detect(frame)
        if not detections:
            if attempts < 10:
                print(f"No detections found, retrying... (Attempt {attempts + 1}/10)")
                self.after(200, lambda: self._match_references(attempts + 1))
                return
            
        matched = self.tracker.init_from_references(detections)
        
        if matched:
            self.start_following()
            sim = matched.get('clip_similarity', 0)
            self.after(0, lambda: (
                self.target_label.configure(
                    text=f"🎯 CLIP MATCH ({sim:.0%})"
                ),
                self.status_label.configure(text="TRACKING", text_color=ACCENT_GREEN),
                self.clip_sim_label.configure(text=f"Similarity:  {sim:.0%}"),
            ))
            
            # Notify Android
            if self.connected_clients and self.ws_loop:
                confirm = json.dumps({
                    "type": "matched",
                    "similarity": round(sim, 2),
                    "message": f"Target locked! ({sim:.0%} match)"
                })
                for c in list(self.connected_clients):
                    try:
                        asyncio.run_coroutine_threadsafe(c.send(confirm), self.ws_loop)
                    except Exception:
                        pass
        else:
            print("CLIP initial match failed. Make sure target is visible to the ESP32 cam.")
            self.after(0, lambda: (
                self.status_label.configure(text="NO MATCH", text_color=ACCENT_ORANGE),
                self.clip_label.configure(text="Target not found in frame. Try again."),
            ))
            # Notify Android
            if self.connected_clients and self.ws_loop:
                confirm = json.dumps({
                    "type": "no_match",
                    "message": "Target not found. Make sure the target is visible."
                })
                for c in list(self.connected_clients):
                    try:
                        asyncio.run_coroutine_threadsafe(c.send(confirm), self.ws_loop)
                    except Exception:
                        pass

    # ─────────────────── Controls ───────────────────

    def start_following(self):
        self.is_wandering = False
        if self.active_mode == "aruco":
            self.is_following = True
            self.aruco_kalman.reset()
            print("[Rover] Following ACTIVATED (Mode: ArUco Marker with Kalman Filter State Estimation)")
            self.status_label.configure(text="TRACKING", text_color=ACCENT_GREEN)
            return

        if self.tracker.target_id or self.identifier.has_references() or self.tracker.bbox:
            self.is_following = True
            print(f"[Rover] Following ACTIVATED (Mode: {self.active_mode}, Target: {self.tracker.target_class} #{self.tracker.target_id})")
            self.status_label.configure(text="TRACKING", text_color=ACCENT_GREEN)
        else:
            print("[Rover] Cannot start following: No target selected yet! Click 'Select Target' or click on an object in the video.")
            self.status_label.configure(text="SELECT TARGET FIRST", text_color=ACCENT_ORANGE)
            self.after(2500, lambda: self.status_label.configure(text="IDLE", text_color=TEXT_PRIMARY) if not self.is_following else None)

    def stop_following(self):
        self.is_following = False
        self.is_wandering = False
        self.aruco_kalman.reset()
        self.stopped_tracking_time = time.time()
        self.comms.send_command(0, 0)
        self.status_label.configure(text="IDLE", text_color=TEXT_PRIMARY)

    def clear_target(self):
        self.stop_following()
        self.tracker.state = "IDLE"
        self.tracker.target_class = None
        self.tracker.target_id = None
        self.tracker.bbox = None
        self.identifier.clear_references()
        self.status_label.configure(text="IDLE", text_color=TEXT_PRIMARY)
        self.target_label.configure(text="No target selected")
        self.clip_label.configure(text="")
        self.clip_refs_label.configure(text="References: 0")
        self.clip_sim_label.configure(text="Similarity:  --")

    # ─────────────────── Video Display ───────────────────

    def update_video(self, frame):
        try:
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            w = self.video_label.winfo_width()
            h = self.video_label.winfo_height()
            if w > 10 and h > 10:
                cv2image = cv2.resize(cv2image, (w, h))

            img = Image.fromarray(cv2image)
            imgtk = ctk.CTkImage(light_image=img, dark_image=img, size=(w, h))
            self.video_label.configure(image=imgtk, text="")
            self.video_label.image = imgtk
        except Exception:
            pass

    # ─────────────────── ML Loop ───────────────────

    def ml_loop(self):
        self.grabber.start()
        fps_time = time.time()
        fps_count = 0
        clip_check_counter = 0

        while self.running:
            frame = self.grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue

            fps_count += 1
            clip_check_counter += 1
            annotated_frame = frame.copy()

            # ── FPS counter ──
            elapsed = time.time() - fps_time
            if elapsed >= 1.0:
                fps_val = fps_count / elapsed
                self.after(0, lambda v=fps_val: self.fps_label.configure(text=f"{v:.1f} FPS"))
                fps_count = 0
                fps_time = time.time()

            # ── Frame Detection ──
            if self.active_mode == "aruco":
                detections = self.aruco_detector.detect(frame)
            else:
                detections = self.detector.detect(frame)

            # (Telemetry is handled asynchronously in the background thread to prevent camera stutter)
            # ── Tracking Logic ──
            if self.is_following:
                if self.active_mode == "aruco":
                    # ── ArUco Following with Discrete Kalman Filter State Estimation ──
                    target_det = None
                    if detections:
                        target_id_cfg = getattr(config, "ARUCO_TARGET_ID", 0)
                        if target_id_cfg is not None:
                            target_det = next((d for d in detections if d.get('id') == target_id_cfg), None)
                        if not target_det and detections:
                            target_det = detections[0]

                    # Step the Kalman Filter:
                    # - If marker measured: corrects position, estimates velocity
                    # - If marker occluded/lost: predicts trajectory forward (dead-reckoning coasting)
                    meas_bbox = target_det['bbox'] if target_det else None
                    estimated_bbox, is_predicted = self.aruco_kalman.step(meas_bbox)

                    if estimated_bbox:
                        left_s, right_s = self.navigator.compute_aruco_speeds(estimated_bbox, is_predicted=is_predicted)
                        left_s, right_s, alert_msg = self.obstacle_avoidance.apply(left_s, right_s)
                        self.comms.send_command(left_s, right_s)

                        tgt_id = target_det.get('id') if target_det else getattr(config, "ARUCO_TARGET_ID", 0)
                        mode_status = "PREDICTING" if is_predicted else "TRACKING"
                        status_color = ACCENT_ORANGE if is_predicted else ACCENT_GREEN

                        if is_predicted:
                            print(f"[Kalman Occlusion Coasting] Target occluded! Predicted bbox {estimated_bbox} -> Speeds: L={left_s}, R={right_s}")
                        else:
                            print(f"[ArUco Kalman] Marker ID: {tgt_id} at bbox {estimated_bbox} -> Speeds: L={left_s}, R={right_s}")

                        active_det_hud = target_det if target_det else {
                            'id': tgt_id,
                            'class': f"ArUco #{tgt_id} (Est)",
                            'bbox': estimated_bbox,
                            'marker_width': estimated_bbox[2]
                        }

                        self.after(0, lambda td=active_det_hud, l=left_s, r=right_s, al=alert_msg, st=mode_status, sc=status_color: (
                            self.target_label.configure(text=f"🎯 {td['class']}"),
                            self.status_label.configure(text=st, text_color=sc if not al else ACCENT_ORANGE),
                            self.tele_left.configure(text=f"Left Motor:   {l}"),
                            self.tele_right.configure(text=f"Right Motor:  {r}")
                        ))

                        if is_predicted:
                            self.aruco_detector.draw_prediction_bbox(annotated_frame, estimated_bbox)
                    else:
                        # Beyond maximum occlusion timeout or not yet initialized
                        self.comms.send_command(0, 0)
                        self.after(0, lambda: (
                            self.status_label.configure(text="SEARCHING", text_color=ACCENT_ORANGE),
                            self.tele_left.configure(text="Left Motor:   0"),
                            self.tele_right.configure(text="Right Motor:  0")
                        ))

                    self.aruco_detector.draw_detections(annotated_frame, detections)
                    self.aruco_detector.draw_hud(
                        annotated_frame,
                        left_s=left_s if estimated_bbox else 0,
                        right_s=right_s if estimated_bbox else 0,
                        target_det=target_det or (active_det_hud if estimated_bbox and is_predicted else None),
                        is_predicted=is_predicted if estimated_bbox else False
                    )

                else:
                    # ── YOLO (Bag / General) Tracking with CLIP ──
                    if self.tracker.state == "TRACKING":
                        success, bbox = self.tracker.update(detections)
                        if not success:
                            self.comms.send_command(0, 0)
                            print(f"[YOLO Tracking] Target not found in frame. Detections count: {len(detections)}. Searching...")
                            self.after(0, lambda: (
                                self.tele_left.configure(text="Left Motor:   0"),
                                self.tele_right.configure(text="Right Motor:  0"),
                                self.status_label.configure(text="SEARCHING", text_color=ACCENT_ORANGE)
                            ))
                        else:
                            left_s, right_s = self.navigator.compute_speeds(bbox)
                            left_s, right_s, alert_msg = self.obstacle_avoidance.apply(left_s, right_s)
                            self.comms.send_command(left_s, right_s)
                            print(f"[YOLO Tracking] Following {self.tracker.target_class} #{self.tracker.target_id} -> Motor Speeds: L={left_s}, R={right_s}")

                            self.after(0, lambda l=left_s, r=right_s, al=alert_msg: (
                                self.tele_left.configure(text=f"Left Motor:   {l}"),
                                self.tele_right.configure(text=f"Right Motor:  {r}"),
                                self.status_label.configure(text="AVOIDING", text_color=ACCENT_ORANGE) if al else None
                            ))

                            bx, by, bw, bh = bbox
                            cv2.rectangle(annotated_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
                            id_str = f" #{self.tracker.target_id}" if self.tracker.target_id else ""
                            cv2.putText(
                                annotated_frame,
                                f"{self.tracker.target_class}{id_str}",
                                (bx, by - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                            )

                    elif self.tracker.state == "SEARCHING":
                        best_match = self.tracker.handle_search(detections)
                        if best_match:
                            self.tracker.init_tracker(frame, best_match, keep_references=True)
                            self.after(0, lambda: self.status_label.configure(
                                text="TRACKING", text_color=ACCENT_GREEN
                            ))
                        elif self.tracker.state == "IDLE":
                            self.stop_following()
                            self.after(0, lambda: self.status_label.configure(
                                text="LOST", text_color=ACCENT_RED
                            ))
            else:
                # When auto-following is not active
                has_manual_keys = bool(self.pressed_keys)

                # Check if 1 minute (60s) has elapsed since tracking was stopped/lost (if enabled)
                now = time.time()
                time_idle = now - self.stopped_tracking_time
                wander_enabled = getattr(config, "ENABLE_AUTONOMOUS_WANDER", False)

                if not has_manual_keys and not getattr(self, "manual_gui_driving", False) and wander_enabled and time_idle >= 60.0:
                    # ── Autonomous Random Wander Mode ──
                    self.is_wandering = True
                    base_spd = 160

                    # Check IR obstacles
                    lb = self.obstacle_avoidance.left_blocked
                    rb = self.obstacle_avoidance.right_blocked

                    if lb and rb:
                        # Obstacle right in front: reverse and pick a new turn direction
                        left_s, right_s = -base_spd, -base_spd
                        self.wander_turn_dir = 1 if (int(now * 10) % 2 == 0) else -1
                        self.wander_turn_until = now + 1.2
                    elif lb:
                        # Left obstacle -> steer right
                        left_s, right_s = base_spd, -base_spd
                        self.wander_turn_until = now + 0.8
                    elif rb:
                        # Right obstacle -> steer left
                        left_s, right_s = -base_spd, base_spd
                        self.wander_turn_until = now + 0.8
                    elif now < self.wander_turn_until:
                        # Continue turning away
                        left_s = base_spd * self.wander_turn_dir
                        right_s = -base_spd * self.wander_turn_dir
                    else:
                        # Drive forward exploring
                        left_s, right_s = base_spd, base_spd

                    self.comms.send_command(left_s, right_s)
                    self.after(0, lambda l=left_s, r=right_s: (
                        self.status_label.configure(text="WANDERING (IR)", text_color=ACCENT_PURPLE),
                        self.tele_left.configure(text=f"Left Motor:   {l}"),
                        self.tele_right.configure(text=f"Right Motor:  {r}")
                    ))
                else:
                    if not has_manual_keys and not getattr(self, "manual_gui_driving", False):
                        self.comms.send_command(0, 0)
                        self.after(0, lambda: (
                            self.tele_left.configure(text="Left Motor:   0"),
                            self.tele_right.configure(text="Right Motor:  0")
                        ))
                    
                # Draw IDLE detections so user knows what they can click on
                if self.active_mode == "aruco":
                    self.aruco_detector.draw_detections(annotated_frame, detections)
                    self.aruco_detector.draw_hud(annotated_frame)
                else:
                    for det in detections:
                        bx, by, bw, bh = det['bbox']
                        cv2.rectangle(annotated_frame, (bx, by), (bx + bw, by + bh), (255, 165, 0), 2)
                        cv2.putText(
                            annotated_frame,
                            f"{det['class']}",
                            (bx, by - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1
                        )



            # Update desktop GUI
            self.after(0, self.update_video, annotated_frame)
            time.sleep(1.0 / config.FPS_TARGET)

    # ─────────────────── WebSocket Server ───────────────────

    def start_ws_server(self):
        async def handler(websocket):
            import websockets as ws_mod

            self.connected_clients.add(websocket)
            self.after(0, lambda: self.conn_indicator.configure(
                text="📱 Mobile Connected", text_color=ACCENT_GREEN
            ))
            print(f"Mobile app connected. ({len(self.connected_clients)} client(s))")

            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        cmd = data.get("cmd")
                        if cmd == "manual_drive":
                            if not self.is_following:
                                self.comms.send_command(
                                    data.get("left", 0), data.get("right", 0)
                                )
                        elif cmd == "set_model":
                            model_choice = data.get("model")
                            if model_choice == "bag":
                                self.active_mode = "bag"
                                config.YOLO_MODEL = "../models/custom_bag.pt"
                                config.YOLO_CONFIDENCE_THRESHOLD = 0.50
                                self.detector = Detector(config.YOLO_MODEL)
                            elif model_choice == "aruco":
                                self.active_mode = "aruco"
                            else:
                                self.active_mode = "general"
                                config.YOLO_MODEL = "../models/yolov8n.pt"
                                config.YOLO_CONFIDENCE_THRESHOLD = 0.30
                                self.detector = Detector(config.YOLO_MODEL)
                            
                            logger.info(f"Switching architecture to: {self.active_mode}")
                            
                        elif cmd == "start":
                            self.start_following()
                        elif cmd == "stop":
                            self.stop_following()

                        elif cmd == "reference_photos":
                            # Android sent reference photos for CLIP matching
                            photos = data.get("photos", [])
                            if photos:
                                logger.info(f"Received {len(photos)} reference photo(s) from Android")
                                threading.Thread(
                                    target=self.handle_reference_photos,
                                    args=(photos,),
                                    daemon=True
                                ).start()
                        elif cmd == "clear_target":
                            self.after(0, self.clear_target)
                    except Exception as e:
                        logger.error(f"WS message error: {e}")
            except ws_mod.exceptions.ConnectionClosed:
                pass
            except Exception as e:
                print(f"WS error: {e}")
            finally:
                self.connected_clients.discard(websocket)
                self.after(0, lambda: self.conn_indicator.configure(
                    text="📱 No Mobile", text_color=TEXT_DIM
                ))
                print("Mobile app disconnected.")

        async def main():
            self.ws_loop = asyncio.get_running_loop()
            async with websockets.serve(handler, "0.0.0.0", 8765):
                await asyncio.Future()

        asyncio.run(main())

    # ─────────────────── Cleanup ───────────────────

    def on_closing(self):
        self.running = False
        self.grabber.stop()
        self.comms.stop()
        self.destroy()
        sys.exit(0)


if __name__ == "__main__":
    app = RoverAppGUI()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
