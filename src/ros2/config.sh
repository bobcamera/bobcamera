#!/bin/bash
# filepath: /workspaces/bobcamera/src/ros2/configure_cameras.sh

CONFIG_FILE="/workspaces/bobcamera/src/ros2/dual_camera_config.yaml"

# Display header
echo "___  ____ ___                                                ";
echo "|__] |  | |__]                                               ";
echo "|__] |__| |__]                                               ";
echo "------------------------";
echo "Universal Object Tracker"
echo "------------------------";
echo "Configuration Wizard"
echo "------------------------";
echo

# Function to validate numeric input
validate_number() {
    local input=$1
    if ! [[ "$input" =~ ^[0-9]+$ ]]; then
        return 1
    fi
    return 0
}

# Ask for number of cameras
while true; do
    read -p "Do you want to use one or two cameras? (1/2): " num_cameras
    if [[ "$num_cameras" == "1" || "$num_cameras" == "2" ]]; then
        break
    else
        echo "Please enter either 1 or 2."
    fi
done

# Camera 1 configuration
echo -e "\n--- Camera 1 Configuration ---"
read -p "Enter name for camera 1 (recording_prefix): " cam1_name

# Camera 1 type selection
echo "Select camera 1 type:"
echo "1) RTSP Stream"
echo "2) USB Camera"
echo "3) Video File"
read -p "Enter selection (1-3): " cam1_type_selection

case $cam1_type_selection in
    1)
        cam1_type="RTSP_STREAM"
        read -p "Enter RTSP URI for camera 1: " cam1_rtsp_uri
        cam1_config="  camera1_node:
    recording_prefix: '$cam1_name' # Prefix name for the recording files
    source_type: 'RTSP_STREAM'        # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
    rtsp_uri: '$cam1_rtsp_uri' # RTSP URI for the camera
    "
        ;;
    2)
        cam1_type="USB_CAMERA"
        while true; do
            read -p "Enter camera ID (usually 0 for first camera): " cam1_id
            if validate_number "$cam1_id"; then
                break
            else
                echo "Please enter a valid number."
            fi
        done
        read -p "Enter camera resolution width (e.g., 640): " cam1_width
        if [[ -z "$cam1_width" ]]; then
            cam1_width=0
        fi
        read -p "Enter camera resolution height (e.g., 480): " cam1_height
        if [[ -z "$cam1_height" ]]; then
            cam1_height=0
        fi
        cam1_config="  camera1_node:
    recording_prefix: '$cam1_name'  # Prefix name for the recording files
    source_type: 'USB_CAMERA'         # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
    camera_id: $cam1_id                      # Id of the camera in the /dev directory if using USB_CAMERA
    usb_camera_resolution: [$cam1_width, $cam1_height] # Resolution for USB Camera [WIDTH, HEIGHT]
    "
        ;;
    3)
        cam1_type="VIDEO_FILE"
        echo "Enter video file path(s) for camera 1 (leave empty to finish):"
        cam1_videos=()
        while true; do
            read -p "Enter video path (or press Enter to finish): " video_path
            if [[ -z "$video_path" ]]; then
                if [[ ${#cam1_videos[@]} -eq 0 ]]; then
                    echo "Please enter at least one video path."
                    continue
                else
                    break
                fi
            else
                cam1_videos+=("$video_path")
            fi
        done
        
        videos_yaml="    videos:                           # List of videos to display in sequence if source_type is VIDEO_FILE"
        for video in "${cam1_videos[@]}"; do
            videos_yaml+=$'\n'"      - '$video'"
        done
        
        cam1_config="  camera1_node:
    recording_prefix: '$cam1_name'    # Prefix name for the recording files
    source_type: 'VIDEO_FILE'         # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
$videos_yaml
    "
        ;;
    *)
        echo "Invalid selection. Exiting."
        exit 1
        ;;
esac

# Camera 2 configuration if needed
if [[ "$num_cameras" == "2" ]]; then
    echo -e "\n--- Camera 2 Configuration ---"
    read -p "Enter name for camera 2 (recording_prefix): " cam2_name

    # Camera 2 type selection
    echo "Select camera 2 type:"
    echo "1) RTSP Stream"
    echo "2) USB Camera"
    echo "3) Video File"
    read -p "Enter selection (1-3): " cam2_type_selection

    case $cam2_type_selection in
        1)
            cam2_type="RTSP_STREAM"
            read -p "Enter RTSP URI for camera 2: " cam2_rtsp_uri
            cam2_config="  camera2_node:
    recording_prefix: '$cam2_name' # Prefix name for the recording files
    source_type: 'RTSP_STREAM'        # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
    rtsp_uri: '$cam2_rtsp_uri' # RTSP URI for the camera"
            ;;
        2)
            cam2_type="USB_CAMERA"
            while true; do
                read -p "Enter camera ID (usually 1 for second camera): " cam2_id
                if validate_number "$cam2_id"; then
                    break
                else
                    echo "Please enter a valid number."
                fi
            done
            read -p "Enter camera resolution width (e.g., 640): " cam2_width
            if [[ -z "$cam2_width" ]]; then
                cam2_width=0
            fi
            read -p "Enter camera resolution height (e.g., 480): " cam2_height
            if [[ -z "$cam2_height" ]]; then
                cam2_height=0
            fi
            cam2_config="  camera2_node:
    recording_prefix: '$cam2_name'  # Prefix name for the recording files
    source_type: 'USB_CAMERA'         # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
    camera_id: $cam2_id                      # Id of the camera in the /dev directory if using USB_CAMERA
    usb_camera_resolution: [$cam2_width, $cam2_height] # Resolution for USB Camera [WIDTH, HEIGHT]"
            ;;
        3)
            cam2_type="VIDEO_FILE"
            echo "Enter video file path(s) for camera 2 (leave empty to finish):"
            cam2_videos=()
            while true; do
                read -p "Enter video path (or press Enter to finish): " video_path
                if [[ -z "$video_path" ]]; then
                    if [[ ${#cam2_videos[@]} -eq 0 ]]; then
                        echo "Please enter at least one video path."
                        continue
                    else
                        break
                    fi
                else
                    cam2_videos+=("$video_path")
                fi
            done
            
            videos_yaml="    videos:                           # List of videos to display in sequence if source_type is VIDEO_FILE"
            for video in "${cam2_videos[@]}"; do
                videos_yaml+=$'\n'"      - '$video'"
            done
            
            cam2_config="  camera2_node:
    recording_prefix: '$cam2_name'    # Prefix name for the recording files
    source_type: 'VIDEO_FILE'         # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
$videos_yaml"
            ;;
        *)
            echo "Invalid selection. Exiting."
            exit 1
            ;;
    esac
else
    # If only one camera, disable camera2
    cam2_config="  camera2_node:
    recording_prefix: 'disabled'        # Prefix name for the recording files
    source_type: 'RTSP_STREAM'        # USB_CAMERA | VIDEO_FILE | RTSP_STREAM
    rtsp_uri: 'rtsp://user:pass@0.0.0.0:554/stream' # RTSP URI for the camera"
fi

# Backup the original file
cp "$CONFIG_FILE" "${CONFIG_FILE}.bak"
echo "Created backup at ${CONFIG_FILE}.bak"

# Update camera2_enabled in basic_variables section
if [[ "$num_cameras" == "2" ]]; then
    sed -i 's/camera2_enabled: false/camera2_enabled: true/g' "$CONFIG_FILE"
else
    sed -i 's/camera2_enabled: true/camera2_enabled: false/g' "$CONFIG_FILE"
fi

# Function to extract sections from the YAML file
extract_section() {
    local file="$1"
    local start_pattern="$2"
    local end_pattern="$3"
    local include_start="$4"
    local include_end="$5"
    
    # Create a temporary file
    local temp_file=$(mktemp)
    
    # Extract the section
    awk -v start="$start_pattern" -v end="$end_pattern" -v inc_start="$include_start" -v inc_end="$include_end" '
    BEGIN { in_section = 0; found_start = 0; }
    $0 ~ start && !found_start { 
        found_start = 1;
        if (inc_start == "true") {
            print $0;
        }
        in_section = 1;
        next;
    }
    $0 ~ end && found_start && in_section { 
        in_section = 0;
        if (inc_end == "true") {
            print $0;
        }
        exit;
    }
    in_section == 1 && found_start { print $0; }
    ' "$file" > "$temp_file"
    
    # Return the content
    cat "$temp_file"
    rm -f "$temp_file"
}

# Create a temporary file
temp_file=$(mktemp)

# Process the file to replace the camera1_node section
# First, let's get the section from the beginning to just before camera1_node
extract_section "$CONFIG_FILE" "^#" "^[[:space:]]*camera1_node:" "true" "false" > "$temp_file"

# Add our new camera1_node configuration
echo "$cam1_config" >> "$temp_file"

# Get the comments and other sections between camera1_node and camera2_node
start_line=$(grep -n "^[[:space:]]*camera1_node:" "$CONFIG_FILE" | head -1 | cut -d: -f1)
end_line=$(grep -n "^[[:space:]]*camera2_node:" "$CONFIG_FILE" | head -1 | cut -d: -f1)

# Extract all lines including comments between the two camera nodes
if [[ -n "$start_line" && -n "$end_line" ]]; then
    awk -v start="$start_line" -v end="$end_line" '
    NR > start && NR < end && $0 ~ /^[[:space:]]*#/ { print $0 }
    ' "$CONFIG_FILE" >> "$temp_file"
fi

# Add our new camera2_node configuration
echo "$cam2_config" >> "$temp_file"

# Now get everything from after camera2_node: to the end of the file
start_line=$(grep -n "^[[:space:]]*camera2_node:" "$CONFIG_FILE" | head -1 | cut -d: -f1)
if [[ -n "$start_line" ]]; then
    end_line=$(wc -l < "$CONFIG_FILE")
    awk -v start="$start_line" -v end="$end_line" '
    BEGIN { found_end = 0; }
    NR > start { 
        # Skip initial lines that are part of camera2_node config
        if (!found_end && ($0 ~ /^[[:space:]]*[a-zA-Z]/ || $0 ~ /^[[:space:]]*$/) && !($0 ~ /^[[:space:]]*(recording_prefix|source_type|rtsp_uri|camera_id|usb_camera_resolution|videos):/) && !($0 ~ /^[[:space:]]*-/)) {
            found_end = 1;
        }
        if (found_end || $0 ~ /^#/) {
            print $0;
        }
    }
    ' "$CONFIG_FILE" >> "$temp_file"
fi

# Move the temporary file to replace the original
mv "$temp_file" "$CONFIG_FILE"

echo -e "\nCamera configuration updated successfully!"
echo "Configuration details:"
echo "- Camera 1: $cam1_name ($cam1_type)"
if [[ "$num_cameras" == "2" ]]; then
    echo "- Camera 2: $cam2_name ($cam2_type)"
    echo "- Camera 2 is ENABLED"
else
    echo "- Camera 2 is DISABLED"
fi
echo -e "\nTo use the new configuration, restart your BOB Camera system."