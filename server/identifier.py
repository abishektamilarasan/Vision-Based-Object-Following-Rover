"""
CLIP-based visual identifier for the Rover follower system.

Uses OpenAI's CLIP (ViT-B/32) to create rich visual embeddings of image crops.
When the user snaps 2-3 reference photos from Android (or selects a target on 
the desktop UI), we extract CLIP embeddings and store them. On each frame, we 
compare every YOLO detection crop against the stored references using cosine 
similarity, finding the best visual match regardless of what YOLO labels it.

Key design: We continuously accumulate reference embeddings while tracking
(every ~30 frames) to build a rich multi-angle visual profile. This makes
re-identification far more robust when the target returns from a different angle.
"""

import torch
import numpy as np
import cv2
import logging
from PIL import Image

logger = logging.getLogger("Identifier")

# Maximum number of reference embeddings to keep (prevents memory bloat)
MAX_REFERENCES = 20

class CLIPIdentifier:
    def __init__(self):
        from transformers import CLIPModel, CLIPProcessor
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        logger.info(f"Loading CLIP ViT-B/32 on {self.device}...")
        
        self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.model.to(self.device)
        self.model.eval()
        
        logger.info("CLIP model loaded successfully.")
        
        # Stored reference embeddings for the target
        self.reference_embeddings = []  # List of numpy arrays (512-D each)
        self.mean_embedding = None      # Average of all references (512-D)
    
    @torch.no_grad()
    def extract_embedding(self, crop_bgr):
        """
        Extract a 512-dimensional CLIP embedding from a BGR image crop.
        Returns a normalized numpy array, or None on failure.
        """
        if crop_bgr is None or crop_bgr.size == 0:
            return None
        
        try:
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(crop_rgb)
            
            inputs = self.processor(images=pil_image, return_tensors="pt")
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            
            # Foolproof extraction bypassing get_image_features quirks
            vision_outputs = self.model.vision_model(**inputs)
            
            # pooled_output is index 1 or .pooler_output
            pooled_output = vision_outputs[1] if isinstance(vision_outputs, tuple) else vision_outputs.pooler_output
            
            # Project to 512-dim embedding space
            features = self.model.visual_projection(pooled_output)
            
            # L2 normalize
            import torch.nn.functional as F
            features = F.normalize(features, p=2, dim=-1)
            return features.cpu().numpy().flatten()
        
        except Exception as e:
            logger.error(f"CLIP embedding extraction failed: {e}")
            return None
    
    def register_reference(self, crop_bgr):
        """
        Register a reference image of the target.
        Accumulates up to MAX_REFERENCES embeddings. When full, drops the oldest.
        """
        emb = self.extract_embedding(crop_bgr)
        if emb is not None:
            self.reference_embeddings.append(emb)
            
            # Drop oldest if we exceed the max
            if len(self.reference_embeddings) > MAX_REFERENCES:
                self.reference_embeddings.pop(0)
            
            # Recompute mean embedding
            self._recompute_mean()
            logger.info(f"Registered reference #{len(self.reference_embeddings)} "
                        f"(total: {len(self.reference_embeddings)})")
            return True
        return False
    
    def _recompute_mean(self):
        """Recompute the mean embedding from all stored references."""
        if self.reference_embeddings:
            self.mean_embedding = np.mean(self.reference_embeddings, axis=0)
            self.mean_embedding = self.mean_embedding / np.linalg.norm(self.mean_embedding)
        else:
            self.mean_embedding = None
    
    def clear_references(self):
        """Clear all stored reference embeddings."""
        self.reference_embeddings = []
        self.mean_embedding = None
        logger.info("Cleared all reference embeddings.")
    
    def identify_best_match(self, detections, threshold=0.60):
        """
        Given a list of detections (each with a 'crop' key), find the one
        that best matches our stored reference embeddings.
        
        Returns (best_detection, similarity_score) or (None, 0.0) if no match.
        
        Default threshold lowered to 0.60 for ESP32-CAM's low-res crops.
        """
        if self.mean_embedding is None or not detections:
            return None, 0.0
        
        best_det = None
        best_sim = -1.0
        
        for det in detections:
            crop = det.get('crop')
            if crop is None or crop.size == 0:
                continue
            
            emb = self.extract_embedding(crop)
            if emb is None:
                continue
            
            # Cosine similarity (both are already L2-normalized)
            sim = float(np.dot(self.mean_embedding, emb))
            det['clip_similarity'] = sim
            
            if sim > best_sim:
                best_sim = sim
                best_det = det
        
        if best_sim >= threshold:
            return best_det, best_sim
        
        return None, best_sim
    
    def has_references(self):
        """Check if any reference images have been registered."""
        return len(self.reference_embeddings) > 0
