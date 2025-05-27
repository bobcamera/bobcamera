#!/bin/bash
# filepath: /workspaces/bobcamera/check_nvidia_encoding.sh

# ANSI color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Checking system for NVIDIA video encoding capabilities...${NC}"

# Check if nvidia-smi is available
if command -v nvidia-smi &> /dev/null; then
    echo -e "${GREEN}✓ NVIDIA driver is installed${NC}"
    nvidia-smi
else
    echo -e "${RED}✗ NVIDIA driver not found${NC}"
    echo "Install NVIDIA driver first"
    exit 1
fi

# Check for NVENC capabilities
echo -e "\n${GREEN}Checking for NVENC capabilities:${NC}"
if [ -f /dev/nvidia-caps/nvidia-cap* ]; then
    echo -e "${GREEN}✓ NVENC capability files found${NC}"
else
    echo -e "${YELLOW}⚠ NVENC capability files not found in /dev/nvidia-caps/${NC}"
fi

# Check for FFmpeg with NVENC support
echo -e "\n${GREEN}Checking FFmpeg for NVENC support:${NC}"
if command -v ffmpeg &> /dev/null; then
    echo -e "${GREEN}✓ FFmpeg is installed${NC}"
    
    # Check if FFmpeg has NVENC support
    if ffmpeg -encoders 2>/dev/null | grep -i nvenc &>/dev/null; then
        echo -e "${GREEN}✓ FFmpeg has NVENC support${NC}"
        echo "Available NVENC encoders:"
        ffmpeg -encoders 2>/dev/null | grep -i nvenc
    else
        echo -e "${RED}✗ FFmpeg does not have NVENC support${NC}"
    fi
else
    echo -e "${RED}✗ FFmpeg not found${NC}"
fi

# Test RTSP connection with increased probe size
echo -e "\n${GREEN}Testing RTSP connection with increased probe values:${NC}"
echo "This will help diagnose the 'not enough frames to estimate rate' warning"

read -p "Enter RTSP URL to test (or press Enter to skip): " rtsp_url

if [ -n "$rtsp_url" ]; then
    echo "Testing connection with increased probesize (10MB) and analyzeduration (10s)..."
    ffprobe -v warning -probesize 10M -analyzeduration 10000000 "$rtsp_url" 2>&1 | grep -v "deprecated"
    
    echo -e "\n${GREEN}Testing stream information:${NC}"
    ffprobe -v warning -select_streams v -show_entries stream=width,height,r_frame_rate,avg_frame_rate -of csv=p=0 "$rtsp_url" 2>&1 | grep -v "deprecated"
    
    echo -e "\n${YELLOW}To resolve the frame rate estimation issue, consider modifying your code:${NC}"
    echo "1. Increase probesize to 10MB:"
    echo '   av_dict_set(&opts, "probesize", "10485760", 0);'
    echo "2. Increase analyzeduration to 10 seconds:"
    echo '   av_dict_set(&opts, "analyzeduration", "10000000", 0);'
fi

echo -e "\n${GREEN}NVIDIA encoding check completed${NC}"