from ultralytics import YOLO
import config
import os
import logging
import torch

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Detector")

class Detector:
    def __init__(self, model_path=config.YOLO_MODEL):
        try:
            self.model = YOLO(model_path)
            logger.info(f"Loaded YOLO model from {model_path}")
        except Exception as e:
            logger.error(f"Error loading YOLO model: {e}")
            self.model = YOLO("yolov8n.pt") # Fallback to download

    def detect(self, frame):
        """
        Runs tracking on a single frame.
        Returns a list of dicts: [{'id': track_id, 'class': name, 'confidence': conf, 'bbox': (x, y, w, h), 'crop': image_crop}]
        """
        device = '0' if torch.cuda.is_available() else 'cpu'
        
        # verbose=False prevents Ultralytics from spamming the console
        # persist=True enables BoT-SORT tracking logic with OSNet Re-ID
        results = self.model.track(frame, persist=True, verbose=False, device=device, tracker="custom_botsort.yaml")
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                track_id = int(box.id[0]) if box.id is not None else None
                
                if conf >= config.YOLO_CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Ensure within bounds
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(frame.shape[1], x2)
                    y2 = min(frame.shape[0], y2)
                    
                    w = x2 - x1
                    h = y2 - y1
                    
                    if w > 0 and h > 0:
                        detections.append({
                            'id': track_id,
                            'class': cls_name,
                            'confidence': conf,
                            'bbox': (x1, y1, w, h),
                            'crop': frame[y1:y2, x1:x2].copy()
                        })
        return detections

    def get_nearest_detection(self, detections, tap_x, tap_y, frame_w, frame_h):
        """
        Finds the detection whose center is closest to the normalized tap coordinates.
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
            
            dist = (center_x - target_x)**2 + (center_y - target_y)**2
            if dist < min_dist:
                min_dist = dist
                best_det = det
                
        return best_det
