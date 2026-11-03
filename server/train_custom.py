import os
import shutil
import random
from ultralytics import YOLO

# Paths provided by you
source_images_dir = r"C:\Users\abish\Desktop\ControlSystems\images_object_following_rover"
source_labels_dir = r"C:\Users\abish\Desktop\ControlSystems\labels_object_following_rover_2026-10-08-06-23-18"

# New dataset structure path
dataset_dir = r"C:\Users\abish\Desktop\ControlSystems\bag_dataset"

# Wipe previous corrupted dataset
shutil.rmtree(dataset_dir, ignore_errors=True)

train_images = os.path.join(dataset_dir, "images", "train")
val_images = os.path.join(dataset_dir, "images", "val")
train_labels = os.path.join(dataset_dir, "labels", "train")
val_labels = os.path.join(dataset_dir, "labels", "val")

# 1. Create Directories
for d in [train_images, val_images, train_labels, val_labels]:
    os.makedirs(d, exist_ok=True)

print("Created dataset folder structure...")

# 2. Get all images
all_images = [f for f in os.listdir(source_images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

print(f"Found {len(all_images)} total images. Using 100% of the dataset for training...")

def copy_data(file_list, dest_img, dest_lbl):
    for img_file in file_list:
        base_name = os.path.splitext(img_file)[0]
        lbl_file = base_name + ".txt"
        
        src_img = os.path.join(source_images_dir, img_file)
        src_lbl = os.path.join(source_labels_dir, lbl_file)
        
        # Only copy if both image and label exist, and the image is perfectly readable
        if os.path.exists(src_lbl):
            if os.path.getsize(src_img) > 0:
                import cv2
                img_test = cv2.imread(src_img)
                if img_test is not None:
                    shutil.copy(src_img, os.path.join(dest_img, img_file))
                    shutil.copy(src_lbl, os.path.join(dest_lbl, lbl_file))
                else:
                    print(f"Warning: Image {img_file} is corrupted (OpenCV cannot read it). Skipping.")
            else:
                print(f"Warning: Image {img_file} is empty (0 bytes). Skipping.")
        else:
            print(f"Warning: Missing label file for image {img_file}. Skipping.")

# 3. Copy files to train and val folders (Using 100% of data for both so YOLO doesn't complain about an empty validation set)
copy_data(all_images, train_images, train_labels)
copy_data(all_images, val_images, val_labels)

# 4. Create the data.yaml file that YOLO needs
yaml_path = os.path.join(dataset_dir, "data.yaml")
yaml_content = f"""path: {dataset_dir}
train: images/train
val: images/val

nc: 1
names: ['black_bag']
"""
with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"Created data.yaml at {yaml_path}")
print("--- Starting YOLOv8 Fine-tuning ---")

import torch
device_type = '0' if torch.cuda.is_available() else 'cpu'

if __name__ == '__main__':
    # 5. Start Training!
    model = YOLO('yolov8n.pt')  # Start from the base nano model
    results = model.train(
        data=yaml_path,
        epochs=50,          # Train for 50 epochs (adjust if you want it faster)
        imgsz=640,          # Image size 
        batch=8,            # Batch size
        device=device_type, # Uses GPU if available, else CPU
        project='runs',     # Where to save the results
        name='black_bag_model',
        workers=0           # Fix for Windows DataLoader crashing
    )
    
    print("\n\nTraining Complete! Your new custom model is saved inside the 'runs/black_bag_model/weights' folder as 'best.pt'")
