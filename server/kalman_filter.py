import time
import numpy as np
import config

class ArucoKalmanFilter:
    """
    Discrete Linear Kalman Filter for ArUco Marker State Estimation & Occlusion Prediction.
    
    State Vector:
        x = [ cx, cy, w, h, vx, vy, vw, vh ]^T
        where (cx, cy) is marker center, (w, h) is bounding box dimensions,
        and (vx, vy, vw, vh) are their respective instantaneous velocities (pixels/second).
        
    Measurement Vector:
        z = [ cx, cy, w, h ]^T
    """
    def __init__(self, process_noise_pos=1.0, process_noise_vel=5.0, measurement_noise=2.0):
        self.state_dim = 8
        self.meas_dim = 4

        # State estimate: [cx, cy, w, h, vx, vy, vw, vh]
        self.x = np.zeros((self.state_dim, 1), dtype=np.float32)

        # State Covariance matrix P
        self.P = np.eye(self.state_dim, dtype=np.float32) * 50.0

        # Measurement matrix H: maps state to measurement (extracts cx, cy, w, h)
        self.H = np.zeros((self.meas_dim, self.state_dim), dtype=np.float32)
        for i in range(self.meas_dim):
            self.H[i, i] = 1.0

        # Measurement noise covariance R
        self.R = np.eye(self.meas_dim, dtype=np.float32) * (measurement_noise ** 2)

        # Baseline noise parameters
        self.q_pos = process_noise_pos
        self.q_vel = process_noise_vel

        self.last_timestamp = None
        self.is_initialized = False

        # Tracking metrics
        self.frames_since_seen = 0
        self.max_coast_seconds = 1.5  # Maximum duration to predict during total occlusion

    def reset(self):
        """Resets the filter state."""
        self.x = np.zeros((self.state_dim, 1), dtype=np.float32)
        self.P = np.eye(self.state_dim, dtype=np.float32) * 50.0
        self.last_timestamp = None
        self.is_initialized = False
        self.frames_since_seen = 0

    def init_state(self, bbox, timestamp=None):
        """
        Initializes the state with the first valid bounding box.
        bbox: (x, y, w, h)
        """
        if timestamp is None:
            timestamp = time.time()

        bx, by, bw, bh = bbox
        cx = float(bx + bw / 2.0)
        cy = float(by + bh / 2.0)

        self.x = np.array([[cx], [cy], [float(bw)], [float(bh)], [0.0], [0.0], [0.0], [0.0]], dtype=np.float32)
        self.P = np.eye(self.state_dim, dtype=np.float32) * 10.0
        self.last_timestamp = timestamp
        self.is_initialized = True
        self.frames_since_seen = 0

    def predict(self, timestamp=None):
        """
        Time Update (Prediction Step):
            x_prior = F * x
            P_prior = F * P * F^T + Q
        """
        if not self.is_initialized:
            return None

        if timestamp is None:
            timestamp = time.time()

        dt = timestamp - self.last_timestamp if self.last_timestamp is not None else (1.0 / config.FPS_TARGET)
        # Clamp dt to reasonable bounds (e.g. 5ms to 200ms)
        dt = max(0.005, min(dt, 0.2))
        self.last_timestamp = timestamp

        # State transition matrix F (constant velocity kinematic model)
        F = np.eye(self.state_dim, dtype=np.float32)
        for i in range(4):
            F[i, i + 4] = dt

        # Process noise covariance Q (discrete approximation)
        Q = np.zeros((self.state_dim, self.state_dim), dtype=np.float32)
        q_p = (self.q_pos * dt) ** 2
        q_v = (self.q_vel * dt) ** 2
        for i in range(4):
            Q[i, i] = q_p
            Q[i + 4, i + 4] = q_v

        # Predict state and covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Return predicted bounding box
        return self.get_bbox()

    def update(self, bbox):
        """
        Measurement Update (Correction Step):
            y = z - H * x_prior  (residual/innovation)
            S = H * P * H^T + R  (innovation covariance)
            K = P * H^T * S^-1   (optimal Kalman gain)
            x_post = x_prior + K * y
            P_post = (I - K * H) * P
        """
        if not self.is_initialized:
            self.init_state(bbox)
            return self.get_bbox()

        bx, by, bw, bh = bbox
        cx = float(bx + bw / 2.0)
        cy = float(by + bh / 2.0)
        z = np.array([[cx], [cy], [float(bw)], [float(bh)]], dtype=np.float32)

        # Innovation
        y = z - (self.H @ self.x)

        # Innovation covariance
        S = (self.H @ self.P @ self.H.T) + self.R

        # Kalman Gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Updated state and covariance
        self.x = self.x + (K @ y)
        I = np.eye(self.state_dim, dtype=np.float32)
        self.P = (I - (K @ self.H)) @ self.P

        self.frames_since_seen = 0
        return self.get_bbox()

    def step(self, detected_bbox, timestamp=None):
        """
        Complete Kalman filter cycle:
        - If detected_bbox is present: predict() then update()
        - If detected_bbox is None (occlusion): predict() only, with velocity dampening
        Returns:
            estimated_bbox: (x, y, w, h)
            is_predicted_only: bool (True if coasting/dead-reckoning through occlusion)
        """
        if timestamp is None:
            timestamp = time.time()

        if not self.is_initialized:
            if detected_bbox is not None:
                self.init_state(detected_bbox, timestamp)
                return self.get_bbox(), False
            return None, False

        # Run prediction step
        self.predict(timestamp)

        if detected_bbox is not None:
            # Measurement available: correct state
            estimated_bbox = self.update(detected_bbox)
            return estimated_bbox, False
        else:
            # No measurement (occluded): coast forward using estimated velocity
            self.frames_since_seen += 1

            # Soft damping on velocity during sustained occlusion so prediction doesn't diverge to infinity
            self.x[4:8] *= 0.95

            # If occluded longer than maximum allowed coast time, expire filter
            elapsed_lost = self.frames_since_seen * (1.0 / config.FPS_TARGET)
            if elapsed_lost > self.max_coast_seconds:
                self.reset()
                return None, False

            return self.get_bbox(), True

    def get_bbox(self):
        """Converts estimated [cx, cy, w, h] state into (x, y, w, h) bounding box format."""
        cx = float(self.x[0, 0])
        cy = float(self.x[1, 0])
        w = max(10.0, float(self.x[2, 0]))
        h = max(10.0, float(self.x[3, 0]))

        x = cx - (w / 2.0)
        y = cy - (h / 2.0)
        return (int(round(x)), int(round(y)), int(round(w)), int(round(h)))

    def get_velocity(self):
        """Returns estimated centroid velocity vector (vx, vy) in pixels/sec."""
        return (float(self.x[4, 0]), float(self.x[5, 0]))

