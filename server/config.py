import os

# Network
ESP32_CAM_URL = "http://10.119.45.16:/stream"
ESP8266_MOTOR_URL = "http://10.53.135.226"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

# Phone Camera (IP Webcam app on Android)
# Install "IP Webcam" from Play Store, start server, use the URL shown
PHONE_CAM_URL = "http://100.120.119.115:8080/video"
CAMERA_SOURCE = "phone"    # "esp32" or "phone"

# Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS_TARGET = 20
FLIP_HORIZONTAL = False  # True for ESP32-CAM (mirrored), False for phone cam

# Architecture / Model Modes: 'bag', 'general', 'aruco'
ACTIVE_MODE = "aruco"
YOLO_MODEL = "../models/custom_bag.pt"
YOLO_CONFIDENCE_THRESHOLD = 0.50

# ArUco Settings (from CS_ArUco_Imp prototype)
ARUCO_DICT = "DICT_4X4_50"
ARUCO_TARGET_ID = 0          # Default target marker ID (or None for any marker)
ARUCO_DEADZONE_X = 35        # Pixel deadzone for steering (narrower = responsive turns)
ARUCO_TARGET_WIDTH = 140     # Desired marker width in pixels at stopping distance
ARUCO_WIDTH_MARGIN = 15      # Tight margin (15px) so rover drives when distance changes
ARUCO_SMOOTH_ALPHA = 0.45    # Responsive smoothing factor (0.45 = fast and smooth)

# Tracking (ByteTrack + CLIP Re-ID)
SEARCH_TIMEOUT_SEC = 10.0

# PID — Angular (turning)
PID_ANGULAR_KP = 0.35        # Lower gain for gentle proportional arcs
PID_ANGULAR_KI = 0.001
PID_ANGULAR_KD = 0.06

# PID — Linear (forward/backward)
PID_LINEAR_KP = 0.55
PID_LINEAR_KI = 0.002
PID_LINEAR_KD = 0.12

# Navigation
MAX_SPEED = 170                  # Increased to deliver enough torque through L298N
MIN_SPEED = 85                   # Breakaway PWM threshold (under 80-90, DC motors stall under load)
DEAD_ZONE = 0.05                 # Error threshold to ignore
TARGET_BBOX_RATIO = 0.15         # Target should occupy ~15% of frame
ENABLE_AUTONOMOUS_WANDER = False  # Set to True to wander after 60s idle

# Obstacle Avoidance (IR sensors — currently disabled)
# IR_DEBOUNCE_MS = 50
# OBSTACLE_STEER_FACTOR = 0.7
# BOTH_BLOCKED_TIMEOUT_SEC = 5.0

# Safety
WATCHDOG_TIMEOUT_MS = 500        # Stop if no command in this time
