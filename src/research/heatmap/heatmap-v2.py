import cv2
import numpy as np
import time

from datetime import datetime
import os
import getopt
import sys
import shutil
from cloud_estimator import CloudEstimator
from day_night_classifier import DayNightEnum, DayNightEstimator

USAGE = 'python heatmap/heatmap-v2.py -d <<directory-containing-allsky and foreground_mask directories>> -m <<mask-filename>>'

ALLSKY = "allsky"
FOREGROUND_MASK = "foreground_mask"
HEATMAPS = "heatmaps"
JSON = "json"
HEATMAP_TIMELAPSE = "heatmap-timelapse.mp4"

def process_dir(recordings_dir, mask_filename=None, resize_factor=1, create_timelapse=True):

    date_time_for_path = datetime.now()
    start_time = time.time()
    enable_json_file_processing = False

    foreground_mask_dir = os.path.join(recordings_dir, FOREGROUND_MASK)
    allsky_dir = os.path.join(recordings_dir, ALLSKY)
    heatmaps_dir = os.path.join(recordings_dir, HEATMAPS)
    json_dir = os.path.join(recordings_dir, JSON)

    if not os.path.isdir(heatmaps_dir):
        os.mkdir(heatmaps_dir)

    allsky_files = os.listdir(allsky_dir)
    foreground_files = os.listdir(foreground_mask_dir)

    if os.path.isdir(json_dir):
        if len(os.listdir(allsky_dir)) > 0:
            enable_json_file_processing = True
            print("Json files found, enable json file processing")

    # Video Files
    # Filter out files that don't have corresponding allsky files
    sorted_video_files = [f for f in foreground_files if f in allsky_files if os.path.isfile(os.path.join(allsky_dir, f))]
    if len(sorted_video_files) <= 0:
        print("No videos found to generate heatmaps from")
        return

    sorted_video_files.sort()    

    foreground_mask_processed_dir = os.path.join(foreground_mask_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(foreground_mask_processed_dir):
        os.mkdir(foreground_mask_processed_dir)

    allsky_processed_dir = os.path.join(allsky_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(allsky_processed_dir):
        os.mkdir(allsky_processed_dir)

    json_processed_dir = os.path.join(json_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(json_processed_dir):
        os.mkdir(json_processed_dir)

    for filename in sorted_video_files:

        base = os.path.basename(filename)
        root_name = os.path.splitext(base)[0]

        foreground_mask_path = os.path.join(foreground_mask_dir, filename)
        allsky_path = os.path.join(allsky_dir, filename)
        json_filename = root_name + ".json"
        json_path = os.path.join(json_dir, json_filename)
        heatmap_filename = os.path.join(heatmaps_dir, root_name + ".png")        

        # generate heatmap file
        process_file(foreground_mask_path, allsky_path, heatmap_filename, mask_filename, resize_factor=resize_factor)

        # move files to processed folders
        foreground_mask_processed_path = os.path.join(foreground_mask_processed_dir, filename)
        shutil.move(foreground_mask_path, foreground_mask_processed_path)
        allsky_processed_path = os.path.join(allsky_processed_dir, filename)
        shutil.move(allsky_path, allsky_processed_path)

        if enable_json_file_processing:
            json_processed_path = os.path.join(json_processed_dir, json_filename)
            if os.path.exists(json_path) and os.path.isfile(json_path):
                shutil.move(json_path, json_processed_path)
            else:
                print(f"Missing json file: {json_filename} for video file {filename}")

    print(f"Generating heatmap images took: {int((time.time() - start_time))} seconds")
    timelapse_start_time = time.time()

    heatmaps_processed_dir = ""
    heatmap_timelapse_filename = ""

    if create_timelapse:
        heatmaps_processed_dir = os.path.join(heatmaps_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
        if not os.path.isdir(heatmaps_processed_dir):
            os.mkdir(heatmaps_processed_dir)

        heatmap_timelapse_filename = os.path.join(heatmaps_dir, f'heatmap-timelapse-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}' + ".mp4")
        process_timelapse(heatmaps_dir, heatmap_timelapse_filename)
        heatmap_files = [f for f in os.listdir(heatmaps_dir) if f.endswith(".jpg") or f.endswith(".png")]
        for heatmap_file in heatmap_files:
            shutil.move(os.path.join(heatmaps_dir, heatmap_file), os.path.join(heatmaps_processed_dir, heatmap_file))

    print(f"Generating heatmap timelapse video took: {int((time.time() - timelapse_start_time))} seconds")
    print(f"Heatmap processing completed in: {int((time.time() - start_time))} seconds and processed {len(sorted_video_files)} videos")

    process_housekeeping(recordings_dir, allsky_processed_dir, foreground_mask_processed_dir, json_processed_dir, heatmaps_processed_dir, 
                         heatmap_timelapse_filename, create_timelapse, enable_json_file_processing)

def process_file(foreground_mask_path, allsky_path, heatmap_filename, mask_filename=None, resize_factor=1):

    print(f"Processing foreground mask video: {foreground_mask_path}, allsky video: {allsky_path}")

    # Load the video
    cap = cv2.VideoCapture(foreground_mask_path)
    cap_allsky = cv2.VideoCapture(allsky_path)
    if not cap.isOpened():
        print(f"Error: Could not open foreground mask video at {foreground_mask_path}.")
        return

    if not cap_allsky.isOpened():
        print(f"Error: Could not open allsky video at {allsky_path}.")
        return

    # Load the mask (important)
    mask = None
    if mask_filename is not None and os.path.exists(mask_filename):
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

    ret_allsky, frame_allsky = cap_allsky.read()
    if not ret_allsky or frame_allsky is None:
        print("Error: Couldn't read the first frame from the allsky video.")
        return

    day_night_estimator = DayNightEstimator.Classifier(95)
    day_night_estimation, average_brightness = day_night_estimator.estimate(frame_allsky)

    cloud_estimator = None
    match day_night_estimation:
        case DayNightEnum.Day:
            cloud_estimator = CloudEstimator.Day()
        case DayNightEnum.Night:
            cloud_estimator = CloudEstimator.Night()

    cloud_estimation = cloud_estimator.estimate(frame_allsky)

    #print(f'Day/Night classifier: {str(day_night_estimation)}, {average_brightness}, cloudy classifier estimation: {cloud_estimation}')

    while True:
        ret, mask_frame = cap.read()

        if not ret:
            break

        gray_mask_frame = cv2.cvtColor(mask_frame, cv2.COLOR_BGR2GRAY)

        if mask is not None:
            #gray_mask_frame *= mask / 255.0
            np.multiply(gray_mask_frame, (mask / 255.0), out=gray_mask_frame, casting='unsafe')

        heatmap += gray_mask_frame

        current_frame = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        progress_percent = (current_frame / total_frames) * 100
        print(f"Processing: {progress_percent:.2f}% done", end="\r")

    cap.release()

    # Normalize the heatmap for visualization
    heatmap_normalized = cv2.normalize(heatmap, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    threshold_value = np.percentile(heatmap_normalized, 99)

    _, thresholded_heatmap = cv2.threshold(heatmap_normalized, threshold_value, 255, cv2.THRESH_TOZERO)

    match day_night_estimation:
        case DayNightEnum.Day:
            heatmap_colored = cv2.applyColorMap(thresholded_heatmap, cv2.COLORMAP_JET) # day
        case DayNightEnum.Night:
            heatmap_colored = cv2.applyColorMap(thresholded_heatmap, cv2.COLORMAP_RAINBOW) # night

    heatmap_bgra = cv2.cvtColor(heatmap_colored, cv2.COLOR_BGR2BGRA)
    heatmap_bgra[thresholded_heatmap == 0, 3] = 0  # Set alpha channel to 0 for transparent regions

    # Overlay the 4-channel heatmap on the max motion frame
    alpha = heatmap_bgra[:, :, 3] / 255.0
    overlay = (heatmap_bgra[:, :, :3] * alpha[:, :, np.newaxis] + frame_allsky * (1 - alpha[:, :, np.newaxis])).astype(np.uint8)

    h, w, _ = overlay.shape
    size = (int(w * resize_factor), int(h * resize_factor))
    overlay = cv2.resize(overlay, size)

    cv2.imwrite(heatmap_filename, overlay)

def process_timelapse(input_folder, output_file, fps=30, resize_factor=1):

    # Get all the files from the input_folder
    files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) if f.endswith(".jpg") or f.endswith(".png")]
    
    # Sort the files based on their names (change the logic if your files aren't named in a sortable way)
    files.sort()

    # Check if there are any files to process
    if not files:
        print("No images found to generate timelapse from!")
        return

    # Find out the frame width and height from the first image
    frame = cv2.imread(files[0])
    h, w, _ = frame.shape
    size = (int(w * resize_factor), int(h * resize_factor))
    
    # Define the codec using VideoWriter_fourcc and create a VideoWriter object
    out = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*'X264'), fps, size)

    for i in range(len(files)):

        base = os.path.basename(files[i])
        file_root_name = os.path.splitext(base)[0]

        img = cv2.imread(files[i])
        if resize_factor != 1:
            img = cv2.resize(img, size)

        # record the root video filename on the image so they can be tied together if needs be
        cv2.putText(img, f"Heatmap for video: {file_root_name}", (25, h-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        #for i in range(int(fps/2)): # 0.5 seconds per frame
        for j in range(fps): # 1 second per frame
            out.write(img)

        print(f"Processed: {i + 1}/{len(files)}", end="\r")

    out.release()

def process_housekeeping(recordings_dir, allsky_working_dir, foreground_mask_working_dir, json_working_dir, heatmaps_working_dir, 
                         heatmap_timelapse_working_filename, enable_timelapse, enable_json):
    
    date_time_for_path = datetime.now()
    processed_dir = os.path.join(recordings_dir, f'{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')

    if not os.path.isdir(processed_dir):
        os.mkdir(processed_dir)

    allsky_processed_dir = os.path.join(processed_dir, ALLSKY)
    if not os.path.isdir(allsky_processed_dir):
        os.mkdir(allsky_processed_dir)
    move_files(allsky_working_dir, allsky_processed_dir)

    foreground_mask_processed_dir = os.path.join(processed_dir, FOREGROUND_MASK)
    if not os.path.isdir(foreground_mask_processed_dir):
        os.mkdir(foreground_mask_processed_dir)
    move_files(foreground_mask_working_dir, foreground_mask_processed_dir)

    if enable_json:
        json_processed_dir = os.path.join(processed_dir, JSON)
        if not os.path.isdir(json_processed_dir):
            os.mkdir(json_processed_dir)
        move_files(json_working_dir, json_processed_dir)

    if enable_timelapse:
        heatmap_processed_dir = os.path.join(processed_dir, HEATMAPS)
        if not os.path.isdir(heatmap_processed_dir):
            os.mkdir(heatmap_processed_dir)
        move_files(heatmaps_working_dir, heatmap_processed_dir)

        heatmap_timelapse_filename = os.path.join(processed_dir, HEATMAP_TIMELAPSE)
        shutil.move(heatmap_timelapse_working_filename, heatmap_timelapse_filename)

def move_files(source_dir, destination_dir):
    files = os.listdir(source_dir)
    for filename in files:
        source_path = os.path.join(source_dir, filename)
        destination_path = os.path.join(destination_dir, filename)
        shutil.move(source_path, destination_path)
    shutil.rmtree(source_dir)

def main(argv):

    # print(f"Open CV Version: {cv2.__version__}")

    try:
        argv = sys.argv[1:] 
        opts, args = getopt.getopt(argv, "h:m:d:", ["help=", "mask=", "directory="])
        #print(f"opts: {opts}, args: {args}")
    except getopt.GetoptError:
        print(USAGE)
        sys.exit(2)

    video_directory = None
    mask_filename = None
    for opt, arg in opts:
        if opt in ['-h', '--help']: 
            print(USAGE)
            sys.exit()
        if opt in ['-m', '--mask']: 
            mask_filename = arg
        if opt in ['-d', '--directory']: 
            video_directory = arg                        

    print(f"video_directory: {video_directory}, mask_filename: {mask_filename}")

    if video_directory is not None:
        recordings_directory = os.path.dirname(video_directory)
        #print(f"recordings_directory: {recordings_directory}")

        if os.path.isdir(recordings_directory):
            process_dir(recordings_directory, mask_filename, resize_factor=0.3)
        else:
            print(USAGE)
            sys.exit()

if __name__ == '__main__':
    main(sys.argv[1:])