import cv2
import time
import subprocess
from datetime import datetime
import os
import getopt
import sys
import shutil

USAGE = 'python heatmap/heatmap-v3.py -d <<directory-containing-allsky, json and heatmap directories>>'

ALLSKY = "allsky"
HEATMAPS = "heatmaps"
JSON = "json"
HEATMAP_TIMELAPSE = "heatmap-timelapse.mp4"

def process_dir(recordings_dir, resize_factor=1, create_timelapse=True, convert_to_mp4=False):

    date_time_for_path = datetime.now()
    enable_json_file_processing = False

    allsky_dir = os.path.join(recordings_dir, ALLSKY)
    heatmaps_dir = os.path.join(recordings_dir, HEATMAPS)
    json_dir = os.path.join(recordings_dir, JSON)

    allsky_files = os.listdir(allsky_dir)

    if os.path.isdir(json_dir):
        if len(os.listdir(allsky_dir)) > 0:
            enable_json_file_processing = True
            print("Json files found, enable json file processing")

    # Video Files
    # Filter out files that don't have corresponding allsky files
    sorted_video_files = [f for f in allsky_files if os.path.isfile(os.path.join(allsky_dir, f))]
    if len(sorted_video_files) <= 0:
        print("No videos found to generate heatmaps from")
        return

    sorted_video_files.sort()    

    allsky_processed_dir = os.path.join(allsky_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(allsky_processed_dir):
        os.mkdir(allsky_processed_dir)

    json_processed_dir = os.path.join(json_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
    if not os.path.isdir(json_processed_dir):
        os.mkdir(json_processed_dir)

    for filename in sorted_video_files:

        base = os.path.basename(filename)
        root_name = os.path.splitext(base)[0]

        allsky_path = os.path.join(allsky_dir, filename)
        json_filename = root_name + ".json"
        json_path = os.path.join(json_dir, json_filename)

        # move files to processed folders
        allsky_processed_path = os.path.join(allsky_processed_dir, filename)
        shutil.move(allsky_path, allsky_processed_path)

        if enable_json_file_processing:
            json_processed_path = os.path.join(json_processed_dir, json_filename)
            if os.path.exists(json_path) and os.path.isfile(json_path):
                shutil.move(json_path, json_processed_path)
            else:
                print(f"Missing json file: {json_filename} for video file {filename}")

    timelapse_start_time = time.time()

    heatmaps_processed_dir = ""
    heatmap_timelapse_filename = ""

    if create_timelapse:
        heatmaps_processed_dir = os.path.join(heatmaps_dir, f'processed-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')
        if not os.path.isdir(heatmaps_processed_dir):
            os.mkdir(heatmaps_processed_dir)

        heatmap_timelapse_filename = os.path.join(heatmaps_dir, f'heatmap-timelapse-{date_time_for_path.strftime("%Y%m%d-%H%M%S")}' + ".mp4")
        process_timelapse(heatmaps_dir, heatmap_timelapse_filename, resize_factor=resize_factor)
        heatmap_files = [f for f in os.listdir(heatmaps_dir) if f.endswith(".jpg") or f.endswith(".png")]
        for heatmap_file in heatmap_files:
            shutil.move(os.path.join(heatmaps_dir, heatmap_file), os.path.join(heatmaps_processed_dir, heatmap_file))

    print(f"Generating heatmap timelapse video took: {int((time.time() - timelapse_start_time))} seconds")

    allsky_mkv_dir = process_housekeeping(recordings_dir, allsky_processed_dir, json_processed_dir, heatmaps_processed_dir, 
                         heatmap_timelapse_filename, create_timelapse, enable_json_file_processing)
    
    if convert_to_mp4:
        process_mkv_to_mp4_conversion(allsky_mkv_dir)

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

def process_housekeeping(recordings_dir, allsky_working_dir, json_working_dir, heatmaps_working_dir, 
                         heatmap_timelapse_working_filename, enable_timelapse, enable_json):
    
    date_time_for_path = datetime.now()
    processed_dir = os.path.join(recordings_dir, f'{date_time_for_path.strftime("%Y%m%d-%H%M%S")}')

    if not os.path.isdir(processed_dir):
        os.mkdir(processed_dir)

    allsky_processed_dir = os.path.join(processed_dir, ALLSKY)
    if not os.path.isdir(allsky_processed_dir):
        os.mkdir(allsky_processed_dir)
    move_files(allsky_working_dir, allsky_processed_dir)

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

    return allsky_processed_dir

def process_mkv_to_mp4_conversion(allsky_mkv_dir, delete_mkv_file=True):

    mkv_files = os.listdir(allsky_mkv_dir)
    for mkv_file in mkv_files:

        mkv_path = os.path.join(allsky_mkv_dir, mkv_file)
        base = os.path.basename(mkv_file)
        root_name = os.path.splitext(base)[0]        
        mp4_filename = root_name + ".mp4"
        mp4_path = os.path.join(allsky_mkv_dir, mp4_filename)        

        #ffmpeg -i mkv_file -codec copy mp4_path
        response = subprocess.call(["ffmpeg", "-i", mkv_path, "-codec", "copy", mp4_path])

        if response == 0:
            print (f'Success: converted {mkv_path} to {mp4_path}, delete file = {delete_mkv_file}')
            if delete_mkv_file:
                os.remove(mkv_path)
        else:
            print (f'Error: converted {mkv_path} to {mp4_path}')

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
    for opt, arg in opts:
        if opt in ['-h', '--help']: 
            print(USAGE)
            sys.exit()
        if opt in ['-d', '--directory']: 
            video_directory = arg                        

    print(f"video_directory: {video_directory}")

    if video_directory is not None:
        recordings_directory = os.path.dirname(video_directory)
        #print(f"recordings_directory: {recordings_directory}")

        if os.path.isdir(recordings_directory):
            process_dir(recordings_directory, resize_factor=0.3)
        else:
            print(USAGE)
            sys.exit()

if __name__ == '__main__':
    main(sys.argv[1:])