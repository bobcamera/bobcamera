import cv2
import numpy as np

# Load the video
cap = cv2.VideoCapture('../../test/fisheye_videos/brad_drone_1.mp4')
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Load the mask (important)
mask = cv2.imread("mask_brad_drone_1.pgm", cv2.IMREAD_GRAYSCALE) 
if mask is None:
    print("Error: Could not load PGM mask.")
    exit()

ret, prev_frame = cap.read()
if not ret:
    print("Error: Could not read frame.")
    exit()

prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
assert prev_gray.shape == mask.shape, "Mask and video dimensions do not match!"

# Initialize an empty heatmap
heatmap = np.zeros_like(prev_gray, dtype=np.float32)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

max_motion_value = 0
max_motion_frame = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(prev_gray, gray)

    masked_diff = diff * (mask / 255.0)
    heatmap += masked_diff

    motion_value = np.sum(masked_diff)
    if motion_value > max_motion_value:
        max_motion_value = motion_value
        max_motion_frame = frame.copy()

    prev_gray = gray

    current_frame = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
    progress_percent = (current_frame / total_frames) * 100
    print(f"Processing: {progress_percent:.2f}% done", end="\r")

cap.release()

# Normalize the heatmap for visualization
heatmap_normalized = cv2.normalize(heatmap, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
threshold_value = np.percentile(heatmap_normalized, 99)

_, thresholded_heatmap = cv2.threshold(heatmap_normalized, threshold_value, 255, cv2.THRESH_TOZERO)
heatmap_colored = cv2.applyColorMap(thresholded_heatmap, cv2.COLORMAP_JET)

heatmap_bgra = cv2.cvtColor(heatmap_colored, cv2.COLOR_BGR2BGRA)
heatmap_bgra[thresholded_heatmap == 0, 3] = 0  # Set alpha channel to 0 for transparent regions

# Overlay the 4-channel heatmap on the max motion frame
alpha = heatmap_bgra[:, :, 3] / 255.0
overlay = (heatmap_bgra[:, :, :3] * alpha[:, :, np.newaxis] + 
          max_motion_frame * (1 - alpha[:, :, np.newaxis])).astype(np.uint8)

cv2.imwrite("overlay.png", overlay)