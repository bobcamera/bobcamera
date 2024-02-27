#!/bin/bash 

ENV_FILE=".env"

show_bob() {
    echo "___  ____ ___                                                ";
    echo "|__] |  | |__]                                               ";
    echo "|__] |__| |__]                                               ";
    echo "---------------------"
    echo "Universal Object Tracker"
    echo "Version 1.0"
    echo "---------------------"
    echo "Configuration Wizard"
    echo "---------------------"
}

show_menu_advanced() {
    clear
    show_bob
    echo "1. Source type"
    [ "$usb_mode" = "'usb'" ] && echo "2. USB Source Selection (Experimental only tested on Linux)" || echo "2. RTSP URL for Fisheye Camera" 
    echo "3. Visualizer"
    echo "4. Recording"
    echo "5. Background Substraction Algorithm"
    echo "6. Tracking Sensitivity"
    echo "7. Masking File Path"
    echo "8. Testing Videos"
    echo "9. Number of Simulation Objects"
    echo "s. Show results"
    echo "q. Quit"
    echo "b. Basic mode"
}

show_menu() {
    clear
    show_bob
    echo "1. Source type"
    [ "$usb_mode" = "'usb'" ] && echo "2. USB Source Selection (Experimental only tested on Linux)" || echo "2. RTSP URL for Fisheye Camera" 
    echo "3. Visualizer"
    echo "4. Recording"
    echo "s. Show results"
    echo "q. Quit"
}


set_config_options() {
    local var_name="$1"
    local initial_message="$2"
    local add_apostrophes="$3" 
    local options=("${@:4}")
    local unconfirmed_result="true"
    local current_value=$(grep "^$var_name=" "$ENV_FILE" | cut -d '=' -f2- | tr -d '"') 
    local formatted_options=""
    if [ "$1" = "BOB_ENABLE_VISUALISER" ]; then
        echo -e "This parameter activates a separate annotated frame stream viewer outside the web user interface\n"
    elif [ "$1" = "BOB_ENABLE_RECORDING" ]; then 
        echo -e "This parameter activates the recording of video files from tracks\n"
    elif [ "$1" = "BOB_SOURCE" ]; then
        echo -e "\nHere you can select the source for the video stream"
        echo "Choose a number from 1 to 6:"
        echo -e "\trtsp(1): Use a rtsp video-stream"
        echo -e "\tusb(2): Use a usb video-stream, this is experimental, as it has only been tested and validated on Linux"
        echo -e "\tvideo(3): pre-recorded video-feed"
        echo -e "\tsimulate(4): simulation of tracker functionality in front of a blank background, with injected blobs"
        echo -e "\trtsp_overlay(5): Like (4) but in front of your rtsp-stream"
        echo -e "\tvideo_overlay(5): Like (4) but in front of the video-stream\n"
    fi

    for i in "${!options[@]}"; do
        if [ "$add_apostrophes" = "true" ]; then
            formatted_options+="\"${options[$i]}\"($((i+1))), "
        else
            formatted_options+="${options[$i]}($((i+1))), "
        fi
    done
    formatted_options=${formatted_options%, *} 

    echo "$initial_message"
    echo "Current value: $current_value"
    echo "Options: $formatted_options"
    read -p "Enter $var_name (1-${#options[@]}): " choice_num
    if ! [[ "$choice_num" =~ ^[1-${#options[@]}]$ ]]; then
        echo "Invalid input. Please enter a number between 1 and ${#options[@]}."
        continue
    fi
    value="${options[$((choice_num-1))]}"

    if [ "$add_apostrophes" = "true" ]; then
        sed -i "s/$var_name=.*/$var_name=\"'$value'\"/" "$ENV_FILE"
        echo "Value in $ENV_FILE replaced with: $var_name=\"'$value'\""
    else
        sed -i "s/$var_name=.*/$var_name=\"$value\"/" "$ENV_FILE"
        echo "Value in $ENV_FILE replaced with: $var_name=\"$value\""
    fi
}


set_rtsp() {
    clear
    show_bob
    echo 'Example Hikvision: "rtsp://user:password@0.0.0.0:554/Streaming/Channels/101"'
    echo 'Example Amcrest and Dahua: "rtsp://user:password@0.0.0.0:554/cam/realmonitor?channel=1&subtype=0"'
    default_url=$(grep "^BOB_RTSP_URL=" "$ENV_FILE" | cut -d '=' -f2-  | tr -d '"')
    [[ $default_url =~ rtsp://([^:]+):([^@]+)@([^:]+):([^/]+)/(.*) ]]
    BOB_RTSP_USER="${BASH_REMATCH[1]}"
    BOB_RTSP_PASSWORD="${BASH_REMATCH[2]}"
    BOB_RTSP_IP="${BASH_REMATCH[3]}"
    BOB_RTSP_PORT="${BASH_REMATCH[4]}"
    BOB_RTSP_URLPATH="${BASH_REMATCH[5]}"

    echo -e "Current URL: $default_url\n\n"
    read -p "Enter the user: " new_user
    BOB_RTSP_USER="${new_user:-$BOB_RTSP_USER}"
    read -p "Enter the password: " new_password
    BOB_RTSP_PASSWORD="${new_password:-$BOB_RTSP_PASSWORD}"
    read -p "Enter the IP address: " new_ip
    BOB_RTSP_IP="${new_ip:-$BOB_RTSP_IP}"
    read -p "Enter the port: " new_port
    BOB_RTSP_PORT="${new_port:-$BOB_RTSP_PORT}"
    read -p "Enter the URL-Path: /" new_urlpath
    BOB_RTSP_URLPATH="${new_urlpath:-$BOB_RTSP_URLPATH}"
    BOB_RTSP_URL="rtsp://$BOB_RTSP_USER:$BOB_RTSP_PASSWORD@$BOB_RTSP_IP:$BOB_RTSP_PORT/$BOB_RTSP_URLPATH"
    echo "Constructed RTSP URL: $BOB_RTSP_URL"  
    sed -i "s|^BOB_RTSP_URL=.*|BOB_RTSP_URL=\"$(echo "$BOB_RTSP_URL" | sed 's/&/\\&/g')\"|" "$ENV_FILE"

}



set_usb() {
    clear
    show_bob
    default_id=$(grep "^BOB_CAMERA_ID=" "$ENV_FILE" | cut -d '=' -f2-  | tr -d '"')
    echo -e "Current Camera ID: $default_id\n\n"
    read -p "Enter the Camera ID: " new_camera_id
    BOB_CAMERA_ID="${new_camera_id:-$BOB_CAMERA_ID}"
    echo "Set Camera ID: $BOB_CAMERA_ID"
    sed -i "s/BOB_CAMERA_ID=.*/BOB_CAMERA_ID=$BOB_CAMERA_ID/" "$ENV_FILE"
}



set_mask_file() {
    clear
    current_value=$(grep "^BOB_TRACKING_MASK_FILE=" "$ENV_FILE" | cut -d '=' -f2-  | tr -d '"')
    read -p "Enter your Masking file with path [$current_value]: " BOB_TRACKING_MASK_FILE
    BOB_TRACKING_MASK_FILE="${BOB_TRACKING_MASK_FILE:-$current_value}"
    sed -i "s|^BOB_TRACKING_MASK_FILE=.*|BOB_TRACKING_MASK_FILE=\"$BOB_TRACKING_MASK_FILE\"|" "$ENV_FILE"
}

set_testing_videos() {
    clear
    current_value=$(grep "^BOB_VIDEOS=" "$ENV_FILE" | cut -d '=' -f2-  | tr -d '"')
    read -p "Enter your Videos with Path [$current_value]: " BOB_VIDEOS
    BOB_VIDEOS="${BOB_VIDEOS:-$current_value}"
    sed -i "s|^BOB_VIDEOS=.*|BOB_VIDEOS=\"$BOB_VIDEOS\"|" "$ENV_FILE"
}

set_number_of_simulated_objects() {
    clear
    current_value=$(grep "^BOB_SIMULATION_NUM_OBJE CTS=" "$ENV_FILE" | cut -d '=' -f2- | tr -d '"')
    read -p "Enter number of simulated objects [$current_value]: " BOB_SIMULATION_NUM_OBJECTS
    BOB_SIMULATION_NUM_OBJECTS="${BOB_SIMULATION_NUM_OBJECTS:-$current_value}"
    sed -i "s|^BOB_SIMULATION_NUM_OBJECTS=.*|BOB_SIMULATION_NUM_OBJECTS=\"$BOB_SIMULATION_NUM_OBJECTS\"|" "$ENV_FILE"
}



advanced_mode="false";
while true; do
    clear
    usb_mode=$(grep "^BOB_SOURCE=" "$ENV_FILE" | cut -d '=' -f2- | tr -d '"')
    if [ "$advanced_mode" = "true" ]; then
        show_menu_advanced    
        read -p "Enter your choice: " choice
        case $choice in
            1) set_config_options "BOB_SOURCE" "Source options:" "true" rtsp usb video simulate rtsp_overlay video_overlay;;
            2) [ "$usb_mode" = "'usb'" ] && set_usb || set_rtsp ;;
            3) set_config_options "BOB_ENABLE_VISUALISER" "Enable Visualizer:" "false" True False;;
            4) set_config_options "BOB_ENABLE_RECORDING" "Enable Recording:" "false" True False;;
            5) set_config_options "BOB_BGS_ALGORITHM" "Background substraction algorithm:"  "false" vibe wmv;;
            6) set_config_options "BOB_TRACKING_SENSITIVITY" "Tracking sensitivity options:" "true"  minimal low medium high;;
            7) set_mask_file ;;
            8) set_testing_videos ;;
            9) set_number_of_simulated_objects ;;
            s) clear; cat ".env"; echo -e "\n\nPress Enter to continue..."; read ;;
            q) clear; echo "Exiting...";  exit ;;
            b) clear; advanced_mode="false";;
            *) echo "Invalid option. Press Enter to continue..."; read ;;
        esac
    else 
        show_menu
        read -p "Enter your choice: " choice
        case $choice in
            1) set_config_options "BOB_SOURCE" "Source options:" "true" rtsp usb video simulate rtsp_overlay video_overlay;;
            2) [ "$usb_mode" = "'usb'" ] && set_usb || set_rtsp ;;
            3) set_config_options "BOB_ENABLE_VISUALISER" "Enable Visualizer:" "false" True False;;
            4) set_config_options "BOB_ENABLE_RECORDING" "Enable Recording:" "false" True False;;
            s) clear; cat ".env"; echo -e "\n\nPress Enter to continue..."; read ;;
            q) clear; echo "Exiting...";  exit ;;
            a) clear; advanced_mode="true";;
            *) echo "Invalid option. Press Enter to continue..."; read ;;
        esac
    fi
done

