import config
import time
import json

class PID:
    def __init__(self, kp, ki, kd, max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.max_integral = max_integral

    def compute(self, error):
        self.integral += error
        
        # Integral windup guard
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
            
        derivative = error - self.prev_error
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.prev_error = 0
        self.integral = 0

class Navigator:
    def __init__(self):
        self.pid_angular = PID(config.PID_ANGULAR_KP, config.PID_ANGULAR_KI, config.PID_ANGULAR_KD)
        self.pid_linear = PID(config.PID_LINEAR_KP, config.PID_LINEAR_KI, config.PID_LINEAR_KD)
        
        # Path recording
        self.recording = False
        self.recorded_path = []
        self.record_start_time = 0

        # ── ArUco smoothing state ──
        self._smooth_left = 0.0       # Exponentially smoothed left output
        self._smooth_right = 0.0      # Exponentially smoothed right output
        self._filtered_width = -1.0   # Low-pass filtered marker width (-1 = uninitialised)

    def compute_speeds(self, bbox):
        """
        Computes left and right motor speeds based on tracking bounding box.
        bbox format: (x, y, w, h)
        """
        if not bbox:
            return 0, 0

        x, y, w, h = bbox
        
        # Calculate horizontal error (-1 to 1)
        center_x = x + w / 2
        frame_center_x = config.FRAME_WIDTH / 2
        horizontal_error = (center_x - frame_center_x) / (config.FRAME_WIDTH / 2)
        
        # Dead zone for angular
        if abs(horizontal_error) < config.DEAD_ZONE:
            horizontal_error = 0

        # Calculate size error for linear speed
        current_ratio = (w * h) / (config.FRAME_WIDTH * config.FRAME_HEIGHT)
        size_error = config.TARGET_BBOX_RATIO - current_ratio
        
        # Dead zone for linear
        if abs(size_error) < config.DEAD_ZONE * 0.5:
            size_error = 0

        angular_vel = self.pid_angular.compute(horizontal_error) * config.MAX_SPEED
        linear_vel = self.pid_linear.compute(size_error) * config.MAX_SPEED

        # Differential drive mapping
        left_speed = linear_vel + angular_vel
        right_speed = linear_vel - angular_vel

        # Add minimum speed offset if non-zero
        if left_speed > 0: left_speed += config.MIN_SPEED
        elif left_speed < 0: left_speed -= config.MIN_SPEED
            
        if right_speed > 0: right_speed += config.MIN_SPEED
        elif right_speed < 0: right_speed -= config.MIN_SPEED

        # Constrain
        left_speed = max(min(int(left_speed), config.MAX_SPEED), -config.MAX_SPEED)
        right_speed = max(min(int(right_speed), config.MAX_SPEED), -config.MAX_SPEED)

        if self.recording:
            self.recorded_path.append({
                "time": time.time() - self.record_start_time,
                "left": left_speed,
                "right": right_speed
            })

        return left_speed, right_speed

    def compute_aruco_speeds(self, bbox, is_predicted=False):
        """
        Computes smooth, proportional left/right motor speeds for ArUco following.
        When is_predicted=True (Kalman coasting during occlusion), forward speeds
        are scaled gracefully to track the predicted trajectory safely.

        bbox format: (x, y, w, h)
        """
        smoothing_alpha = getattr(config, "ARUCO_SMOOTH_ALPHA", 0.45)

        if not bbox:
            # Smoothly ramp output to zero when marker is lost
            self._smooth_left *= 0.6
            self._smooth_right *= 0.6
            if abs(self._smooth_left) < 3 and abs(self._smooth_right) < 3:
                self._smooth_left = 0.0
                self._smooth_right = 0.0
            return int(self._smooth_left), int(self._smooth_right)

        x, y, w, h = bbox
        if w <= 0 or h <= 0:
            return 0, 0

        frame_w = float(config.FRAME_WIDTH)
        frame_center_x = frame_w / 2.0

        # When using Kalman filter, the marker width is already state-estimated
        filtered_w = float(w)

        # ── 1. Angular / Horizontal Error (-1.0 … +1.0) ──
        center_x = x + (w / 2.0)
        horizontal_offset = center_x - frame_center_x
        deadzone_px = float(getattr(config, "ARUCO_DEADZONE_X", 35))

        if abs(horizontal_offset) < deadzone_px:
            angular_error = 0.0
        else:
            sign = 1.0 if horizontal_offset > 0 else -1.0
            angular_error = (horizontal_offset - sign * deadzone_px) / (frame_center_x - deadzone_px)
            angular_error = max(min(angular_error, 1.0), -1.0)

        # ── 2. Linear / Distance Error ──
        target_width = float(getattr(config, "ARUCO_TARGET_WIDTH", 140))
        width_margin = float(getattr(config, "ARUCO_WIDTH_MARGIN", 15))
        width_error = target_width - filtered_w

        if abs(width_error) < width_margin:
            linear_error = 0.0
        else:
            sign = 1.0 if width_error > 0 else -1.0
            linear_error = (width_error - sign * width_margin) / target_width
            linear_error = max(min(linear_error, 1.0), -1.0)

        # ── Complete stop when both errors are inside deadzone ──
        if abs(angular_error) < 0.01 and abs(linear_error) < 0.01:
            self.pid_angular.reset()
            self.pid_linear.reset()
            # Ramp down smoothly instead of snapping to zero
            self._smooth_left *= 0.5
            self._smooth_right *= 0.5
            if abs(self._smooth_left) < 3:
                self._smooth_left = 0.0
            if abs(self._smooth_right) < 3:
                self._smooth_right = 0.0
            return int(self._smooth_left), int(self._smooth_right)

        # ── PID outputs ──
        angular_output = self.pid_angular.compute(angular_error)
        linear_output = self.pid_linear.compute(linear_error)

        # Scale to motor speed range
        linear_vel = linear_output * config.MAX_SPEED * 1.0
        angular_vel = angular_output * config.MAX_SPEED * 0.45

        # During occlusion dead-reckoning, scale speed gently for safety
        if is_predicted:
            linear_vel *= 0.85
            angular_vel *= 0.85

        # ── Forward bias when turning at target distance ──
        # If the marker is off to the side but we're already at the right distance,
        # inject a gentle forward speed so the rover drives an arc toward the marker
        # instead of spinning in place (which was the old pure-pivot behavior).
        if abs(angular_error) > 0.05 and abs(linear_vel) < config.MIN_SPEED:
            linear_vel = config.MIN_SPEED * 0.8

        # ── Differential drive (always curved arcs, never pure spin) ──
        left_speed = linear_vel + angular_vel
        right_speed = linear_vel - angular_vel

        # Add minimum speed offset only when commanding significant movement
        if left_speed > 5:
            left_speed += config.MIN_SPEED
        elif left_speed < -5:
            left_speed -= config.MIN_SPEED
        else:
            left_speed = 0

        if right_speed > 5:
            right_speed += config.MIN_SPEED
        elif right_speed < -5:
            right_speed -= config.MIN_SPEED
        else:
            right_speed = 0

        # Constrain to MAX_SPEED limits
        left_speed = max(min(float(left_speed), config.MAX_SPEED), -config.MAX_SPEED)
        right_speed = max(min(float(right_speed), config.MAX_SPEED), -config.MAX_SPEED)

        # ── Exponential smoothing (prevents jerky frame-to-frame jumps) ──
        self._smooth_left = smoothing_alpha * left_speed + (1.0 - smoothing_alpha) * self._smooth_left
        self._smooth_right = smoothing_alpha * right_speed + (1.0 - smoothing_alpha) * self._smooth_right

        left_out = int(self._smooth_left)
        right_out = int(self._smooth_right)

        # Ensure nonzero commands provide at least breakaway speed
        if 0 < abs(left_out) < config.MIN_SPEED:
            left_out = config.MIN_SPEED if left_out > 0 else -config.MIN_SPEED
        if 0 < abs(right_out) < config.MIN_SPEED:
            right_out = config.MIN_SPEED if right_out > 0 else -config.MIN_SPEED

        if self.recording:
            self.recorded_path.append({
                "time": time.time() - self.record_start_time,
                "left": left_out,
                "right": right_out
            })

        return left_out, right_out

    def reset_pids(self):
        self.pid_angular.reset()
        self.pid_linear.reset()
        
    def start_recording(self):
        self.recording = True
        self.recorded_path = []
        self.record_start_time = time.time()
        
    def stop_recording(self, save_path="recorded_path.json"):
        self.recording = False
        try:
            with open(save_path, "w") as f:
                json.dump(self.recorded_path, f)
            print(f"Path saved to {save_path} with {len(self.recorded_path)} steps.")
        except Exception as e:
            print(f"Error saving path: {e}")
            
    def load_path(self, load_path="recorded_path.json"):
        try:
            with open(load_path, "r") as f:
                self.recorded_path = json.load(f)
            return self.recorded_path
        except Exception as e:
            print(f"Error loading path: {e}")
            return []
