#!/bin/bash 

YAML_FILE="${USER}.yaml"

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

show_menu() {
    clear
    show_bob
    echo "1. Source type"
    [ "$usb_mode" = "'usb'" ] && echo "2. USB Source Selection (Experimental only tested on Linux)" || echo "2. RTSP URL for Fisheye Camera" 
    echo "3. Recording"
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
    if [ "$1" = "BOB_ENABLE_RECORDING" ]; then 
        echo -e "This parameter activates the recording of video files from tracks\n"
    elif [ "$1" = "BOB_SOURCE" ]; then
        echo -e "\nHere you can select the source for the video stream"
        echo "Choose a number from 1 to 5:"
        echo -e "\trtsp(1): Use a rtsp video-stream"
        echo -e "\tusb(2): Use a usb video-stream, this is experimental, as it has only been tested and validated on Linux"
        echo -e "\tvideo(3): pre-recorded video-feed"
        echo -e "\trtsp_overlay(4): Simulation of tracker functionality in front of your rtsp-stream, with injected blobs"
        echo -e "\tvideo_overlay(5): Simulation of tracker functionality in front of the video-stream, with injected blobs\n"
    fi

    for i in "${!options[@]}"; do
        if [ "$add_apostrophes" = "true" ]; then
            formatted_options+="\"${options[$i]}\"($((i+1))), "
        else
            formatted_options+="${options[$i]}($((i+1))), "
        fi
    done
    formatted_options=${formatted_options%, *} 
    while true; do 
        echo "$initial_message"
        echo "Current value: $current_value"
        echo "Options: $formatted_options"
        read -p "Enter $var_name (1-${#options[@]}): " choice_num
        if ! [[ "$choice_num" =~ ^[1-${#options[@]}]$ ]]; then
            echo "Invalid input. Please enter a number between 1 and ${#options[@]}."
            continue
        else 
            value="${options[$((choice_num-1))]}"

            if [ "$add_apostrophes" = "true" ]; then
                sed -i "s/$var_name=.*/$var_name=\"'$value'\"/" "$ENV_FILE"
                echo "Value in $ENV_FILE replaced with: $var_name=\"'$value'\""
            else
                sed -i "s/$var_name=.*/$var_name=\"$value\"/" "$ENV_FILE"
                echo "Value in $ENV_FILE replaced with: $var_name=\"$value\""
            fi
            break
        fi
    done

}

set_rtsp() {
    clear
    show_bob
    echo 'Example Hikvision: rtsp://peter:pAsSwOrD123427@12.8.9.3:554/Streaming/Channels/101'
    echo 'Example Amcrest and Dahua: rtsp://user:password@0.0.0.0:554/cam/realmonitor?channel=1&subtype=0'
    echo 'Pattern: rtsp://{name of the user}:{password of user}@{IP-Address}:{Port Number}/{Path}'
    echo '{} is just a placeholder in the Pattern, just give the value without additional symbols like {} or "" or the like'

    default_url=$(grep "^BOB_RTSP_URL=" "$ENV_FILE" | cut -d '=' -f2-  | tr -d '"')
    [[ $default_url =~ rtsp://([^:]+):([^@]+)@([^:]+):([^/]+)/(.*) ]]
    BOB_RTSP_USER="${BASH_REMATCH[1]}"
    BOB_RTSP_PASSWORD="${BASH_REMATCH[2]}"
    BOB_RTSP_IP="${BASH_REMATCH[3]}"
    BOB_RTSP_PORT="${BASH_REMATCH[4]}"
    BOB_RTSP_URLPATH="${BASH_REMATCH[5]}"

    echo -e "Current URL: $default_url\n\n"
    read -p "Enter the name of the user (e.g. peter): " new_user
    BOB_RTSP_USER="${new_user:-$BOB_RTSP_USER}"
    read -p "Enter the password of the user (e.g. pAsSwOrD1234): " new_password
    BOB_RTSP_PASSWORD="${new_password:-$BOB_RTSP_PASSWORD}"
    read -p "Enter the IP address (e.g. 12.8.9.3): " new_ip
    BOB_RTSP_IP="${new_ip:-$BOB_RTSP_IP}"
    read -p "Enter the port (e.g. 554): " new_port
    BOB_RTSP_PORT="${new_port:-$BOB_RTSP_PORT}"
    read -p "Enter the URL-Path (e.g. Streaming/Channels/101): /" new_urlpath
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

if [ ! -f "$YAML_FILE" ]; then
    cp ex_config_video.yaml "$YAML_FILE"
    echo "$YAML_FILE has been created."
else
    echo "$YAML_FILE already exists."
fi

while true; do
    clear
    usb_mode=$(grep "^BOB_SOURCE=" "$ENV_FILE" | cut -d '=' -f2- | tr -d '"')
    show_menu
    read -p "Select a number: " choice
    case $choice in
        1) set_config_options "source_type" "Source options:" "true" rtsp usb video simulate rtsp_overlay video_overlay;read -rp "Press Enter to continue...";;
        2) [ "$usb_mode" = "'usb'" ] && set_usb || set_rtsp ;read -rp "Press Enter to continue...";;
        3) set_config_options "BOB_ENABLE_RECORDING" "Enable Recording:" "false" True False;read -rp "Press Enter to continue...";;
        q) clear; exit ;;
        *) echo "Invalid option. Press Enter to continue..."; read ;;
    esac
    
done
