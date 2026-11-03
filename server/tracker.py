import time
import config

# How often to add a new CLIP reference while tracking (in frames).
# Lower = richer profile but more GPU usage. 30 frames ≈ 1.5 sec at 20 FPS.
CLIP_ACCUMULATE_INTERVAL = 30


class Tracker:
    def __init__(self, identifier=None):
        self.state = "IDLE" # IDLE, TRACKING, SEARCHING
        self.target_class = None
        self.target_id = None
        self.last_seen_time = 0
        self.bbox = None # (x, y, w, h)
        self.identifier = identifier  # CLIPIdentifier instance
        self.frames_since_clip = 0    # Counter for periodic CLIP accumulation

    def init_tracker(self, frame, detection, keep_references=False):
        """
        Initializes the tracker by locking onto the specific YOLO track_id.
        
        If keep_references=False (default for first selection), clears old CLIP
        references and registers the new crop.
        If keep_references=True (used during re-identification), preserves the
        existing CLIP memory so we don't lose our multi-angle profile.
        """
        self.target_id = detection.get('id')
        self.target_class = detection['class']
        self.bbox = detection['bbox']
        self.frames_since_clip = 0
        
        if getattr(config, "ACTIVE_MODE", "bag") != "aruco" and self.identifier and 'crop' in detection:
            if not keep_references:
                # Fresh target — clear old references
                self.identifier.clear_references()
                self.identifier.register_reference(detection['crop'])
            else:
                # Re-identification — add this crop to the existing profile
                self.identifier.register_reference(detection['crop'])
        
        self.state = "TRACKING"
        self.last_seen_time = time.time()
            
        print(f"{'Re-locked' if keep_references else 'Started following'}: "
              f"{self.target_class} (ID: {self.target_id}) at {self.bbox}")

    def init_from_references(self, detections):
        """
        Called after the user sends reference photos from Android.
        Finds the best CLIP match among current detections and starts tracking it.
        Returns the matched detection or None.
        """
        if not self.identifier or not self.identifier.has_references():
            print("No references found in identifier.")
            return None
            
        if not detections:
            print("No YOLO detections in the current frame to match against.")
            return None
        
        best_det, sim = self.identifier.identify_best_match(detections, threshold=0.45)
        
        if best_det:
            self.target_id = best_det.get('id')
            self.target_class = best_det['class']
            self.bbox = best_det['bbox']
            self.frames_since_clip = 0
            self.state = "TRACKING"
            self.last_seen_time = time.time()
            print(f"Initial CLIP match SUCCESS! Similarity: {sim:.0%}")
            return best_det
        else:
            print(f"Initial CLIP match FAILED! Highest similarity found was {sim:.0%} (needs 45%)")
            return None

    def update(self, detections):
        """
        Updates the tracker based on the latest YOLO detections.
        Uses ByteTrack ID matching first, then CLIP fallback if ID is lost.
        Also periodically accumulates CLIP references for a richer profile.
        """
        if self.state != "TRACKING":
            return False, None

        self.frames_since_clip += 1

        # Primary: ByteTrack ID matching (fast, per-frame)
        matched_det = None
        if self.target_id is not None:
            for det in detections:
                if det.get('id') == self.target_id:
                    matched_det = det
                    break

        if matched_det:
            self.bbox = matched_det['bbox']
            self.target_class = matched_det['class']
            self.last_seen_time = time.time()

            # Periodically accumulate CLIP references from successful tracks.
            # This builds a multi-angle visual profile so re-identification
            # works even when the target returns from a different angle.
            if (getattr(config, "ACTIVE_MODE", "bag") != "aruco"
                    and self.identifier and self.frames_since_clip >= CLIP_ACCUMULATE_INTERVAL
                    and 'crop' in matched_det):
                self.identifier.register_reference(matched_det['crop'])
                self.frames_since_clip = 0

            return True, self.bbox

        # Secondary: CLIP Re-Identification (when ByteTrack loses the ID - only in bag/general mode)
        if getattr(config, "ACTIVE_MODE", "bag") != "aruco" and self.identifier and self.identifier.has_references():
            best_det, sim = self.identifier.identify_best_match(detections, threshold=0.70)
            if best_det:
                old_id = self.target_id
                self.target_id = best_det.get('id')
                self.target_class = best_det['class']
                self.bbox = best_det['bbox']
                self.last_seen_time = time.time()
                self.frames_since_clip = 0
                # Add this new angle to our profile
                if 'crop' in best_det:
                    self.identifier.register_reference(best_det['crop'])
                print(f"CLIP Re-ID: {old_id} → {self.target_id} ({sim:.0%} match)")
                return True, self.bbox

        # Totally lost
        self.state = "SEARCHING"
        print(f"Lost target {self.target_class} (ID: {self.target_id}). Searching...")
        return False, None

    def handle_search(self, detections):
        """
        Called when in SEARCHING state. Uses CLIP to re-identify the target.
        Does NOT clear references — our accumulated multi-angle profile is
        the key to finding the target again.
        """
        if self.state != "SEARCHING":
            return False

        if time.time() - self.last_seen_time > config.SEARCH_TIMEOUT_SEC:
            self.state = "IDLE"
            self.target_class = None
            self.target_id = None
            self.bbox = None
            print("Search timeout. Returning to IDLE.")
            return False

        # Try ByteTrack native ID match first (in case it re-assigned the same ID)
        for det in detections:
            if self.target_id is not None and det.get('id') == self.target_id:
                print(f"ByteTrack Re-identified {self.target_class}. Resuming tracking.")
                self.target_class = det['class']
                return det

        # CLIP Re-Identification — the main recovery mechanism
        if self.identifier and self.identifier.has_references():
            best_det, sim = self.identifier.identify_best_match(detections, threshold=0.50)
            if best_det:
                print(f"CLIP Re-identified target with {sim:.0%} similarity. Resuming tracking.")
                return best_det
            else:
                print(f"CLIP searching... (best sim: {sim:.0%})")
            
        return False
