import cv2
import cv2.aruco as aruco
import numpy as np
import logging
import config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ArucoDetector")

ARUCO_DICT_MAP = {
    "DICT_4X4_50": aruco.DICT_4X4_50,
    "DICT_4X4_100": aruco.DICT_4X4_100,
    "DICT_4X4_250": aruco.DICT_4X4_250,
    "DICT_5X5_50": aruco.DICT_5X5_50,
    "DICT_5X5_100": aruco.DICT_5X5_100,
    "DICT_6X6_50": aruco.DICT_6X6_50,
}


class ArucoDetector:
    """
    ArUco marker detector conforming to the Detector interface.
    Integrates the prototype logic from CS_ArUco_Imp.
    """
    def __init__(self, dict_name=config.ARUCO_DICT, target_id=config.ARUCO_TARGET_ID):
        dict_type = ARUCO_DICT_MAP.get(dict_name, aruco.DICT_4X4_50)
        self.dict_name = dict_name
        self.target_id = target_id
        
        self.aruco_dict = aruco.getPredefinedDictionary(dict_type)
        self.aruco_params = aruco.DetectorParameters()
        
        # Check if ArucoDetector class is available in OpenCV (v4.7+)
        if hasattr(aruco, "ArucoDetector"):
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None
            
        logger.info(f"Initialized ArucoDetector with dictionary {dict_name}, default target_id: {target_id}")

    def detect(self, frame):
        """
        Runs ArUco marker detection on frame.
        Returns a list of dicts conforming to standard detector output:
        [{'id': int, 'class': str, 'confidence': float, 'bbox': (x, y, w, h), 'crop': ndarray, 'corners': ndarray, 'center': (mx, my), 'width': float}]
        """
        if frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        detections = []
        if ids is not None and len(ids) > 0:
            flat_ids = np.ravel(ids)
            for i in range(len(flat_ids)):
                detected_id = int(flat_ids[i])
                c = corners[i][0] # 4 points (x, y)
                
                x_min = int(np.min(c[:, 0]))
                x_max = int(np.max(c[:, 0]))
                y_min = int(np.min(c[:, 1]))
                y_max = int(np.max(c[:, 1]))
                
                # Clip to frame boundary
                x1 = max(0, x_min)
                y1 = max(0, y_min)
                x2 = min(frame.shape[1], x_max)
                y2 = min(frame.shape[0], y_max)
                w = max(1, x2 - x1)
                h = max(1, y2 - y1)
                
                mx = int(np.mean(c[:, 0]))
                my = int(np.mean(c[:, 1]))
                m_width = float(np.linalg.norm(c[0] - c[1]))
                
                crop = frame[y1:y2, x1:x2].copy() if (w > 0 and h > 0) else np.zeros((10, 10, 3), dtype=np.uint8)
                
                detections.append({
                    'id': detected_id,
                    'class': f"ArUco #{detected_id}",
                    'confidence': 1.0,
                    'bbox': (x1, y1, w, h),
                    'crop': crop,
                    'corners': c,
                    'center': (mx, my),
                    'marker_width': m_width
                })

        return detections

    def draw_detections(self, frame, detections):
        """
        Draws ArUco marker borders, centers, and labels on frame.
        """
        for det in detections:
            c = det.get('corners')
            if c is not None:
                cv2.polylines(frame, [c.astype(np.int32)], True, (0, 255, 0), 2)
            
            center = det.get('center')
            if center is not None:
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
            x, y, w, h = det['bbox']
            label = f"{det['class']} (W:{int(det.get('marker_width', w))})"
            cv2.putText(
                frame,
                label,
                (x, max(15, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

    def get_nearest_detection(self, detections, tap_x, tap_y, frame_w, frame_h):
        """
        Finds the detection closest to normalized coordinates (tap_x, tap_y).
        """
        if not detections:
            return None

        target_x = int(tap_x * frame_w)
        target_y = int(tap_y * frame_h)

        best_det = None
        min_dist = float('inf')

        for det in detections:
            x, y, w, h = det['bbox']
            center_x = x + w / 2
            center_y = y + h / 2

            dist = (center_x - target_x) ** 2 + (center_y - target_y) ** 2
            if dist < min_dist:
                min_dist = dist
                best_det = det

        return best_det

    def draw_hud(self, frame, left_s=None, right_s=None, target_det=None, is_predicted=False):
        """
        Draws the visual guidelines from CS_ArUco_Imp:
        center deadzone lines, target ID, width, and Dual PID motor output speeds.
        Also displays Kalman Filter state (MEASURED vs PREDICTED / OCCLUSION).
        """
        h, w = frame.shape[:2]
        center_x = w // 2
        deadzone = getattr(config, "ARUCO_DEADZONE_X", 35)

        # Draw center deadzone lines
        cv2.line(frame, (center_x - deadzone, 0), (center_x - deadzone, h), (200, 200, 200), 1)
        cv2.line(frame, (center_x + deadzone, 0), (center_x + deadzone, h), (200, 200, 200), 1)

        if target_det:
            m_w = int(target_det.get('marker_width', target_det['bbox'][2]))
            speed_str = f" | PID: L={left_s} R={right_s}" if (left_s is not None and right_s is not None) else ""
            
            if is_predicted:
                hud_text = f"KALMAN PREDICTION [OCCLUSION] ID:{target_det['id']} W:{m_w}{speed_str}"
                color = (0, 165, 255) # Orange for predicted state
            else:
                hud_text = f"KALMAN TRACKED ID:{target_det['id']} W:{m_w}{speed_str}"
                color = (0, 255, 0)   # Green for verified measurement

            cv2.putText(
                frame,
                hud_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

    def draw_prediction_bbox(self, frame, bbox):
        """
        Draws a distinctive dashed/highlighted bounding box representing the
        Kalman filter's predicted position during occlusion.
        """
        if not bbox:
            return
        x, y, w, h = bbox
        # Draw bounding rectangle in safety orange with double border
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)
        cv2.putText(
            frame,
            "ESTIMATED PATH (KALMAN)",
            (x, max(15, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 140, 255),
            2
        )
        # Centroid crosshair
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(frame, (cx, cy), 4, (0, 140, 255), -1)
