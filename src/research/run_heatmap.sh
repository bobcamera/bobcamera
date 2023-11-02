#!/bin/bash

export PYTHONPATH="${PYTHONPATH}:heatmap/"
#python3 heatmap/heatmap.py  -d../ros2/assets/recordings/allsky/
python3 heatmap/heatmap-v2.py  -d ../ros2/assets/recordings/ -m ../ros2/assets/masks/mask.jpg