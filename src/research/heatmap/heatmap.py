import cv2
import numpy as np

from datetime import datetime
import os
import getopt
import sys
import shutil
from pathlib import Path

USAGE = 'python heatmap/heatmap.py -d <<directory-of-videos>> -m <<mask-filename>>'

def process_dir(input_dir, mask_filename=None):

    processed_dir = os.path.join(input_dir, f'heatmaps-{datetime.now().strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(processed_dir):
        os.mkdir(processed_dir)

    sorted_files = os.listdir(input_dir)
    sorted_files.sort()

    for filename in sorted_files:
        full_path = os.path.join(input_dir, filename)

        if os.path.isdir(full_path):
            continue

        base = os.path.basename(filename)
        root_name = os.path.splitext(base)[0]
        heatmap_filename = os.path.join(processed_dir, root_name + ".png")

        process_file(full_path, heatmap_filename, mask_filename)
        processed_path = os.path.join(processed_dir, filename)
        shutil.move(full_path, processed_path)

def process_file(video_path, heatmap_filename, mask_filename=None):

    print(f"processing file: {video_path}")

    # Load the video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()

    # Load the mask (important)
    mask = None
    if mask_filename is not None and Path.exists(mask_filename):
        mask = cv2.imread(mask_filename, cv2.IMREAD_GRAYSCALE) 
        if mask is None:
            print("Error: Could not load PGM mask.")
            exit()

    ret, prev_frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        exit()

    prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

    if mask is not None:
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

        if mask is not None:
            masked_diff = diff * (mask / 255.0)
            heatmap += masked_diff
            motion_value = np.sum(masked_diff)
        else:
            heatmap += diff
            motion_value = np.sum(diff)

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
    overlay = (heatmap_bgra[:, :, :3] * alpha[:, :, np.newaxis] + max_motion_frame * (1 - alpha[:, :, np.newaxis])).astype(np.uint8)

    cv2.imwrite(heatmap_filename, overlay)

def main(argv):

    print(f"Open CV Version: {cv2.__version__}")

    try:
        opts, args = getopt.getopt(argv, "hd:", [])
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)

    video_directory = None
    mask_filename = None
    for opt, arg in opts:
        if opt == '-h':
            print(USAGE)
            sys.exit()
        if opt == '-m':
            mask_filename = arg
        if opt == '-d':
            video_directory = arg                        

    print(f"video_directory: {video_directory}, mask_filename: {mask_filename}")

    if video_directory is not None:
        process_dir(video_directory, mask_filename)

if __name__ == '__main__':
    main(sys.argv[1:])