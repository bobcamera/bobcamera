#!/bin/bash
echo "===== NVIDIA Encoding Diagnostic Tool ====="
echo -e "\nSystem Information:"
cat /etc/os-release
uname -a

echo -e "\nNVIDIA Driver Information:"
nvidia-smi 2>/dev/null || echo "NVIDIA driver not available"

echo -e "\nFFmpeg Hardware Acceleration:"
ffmpeg -hide_banner -hwaccels

echo -e "\nNVIDIA Encoder Support in FFmpeg:"
ffmpeg -hide_banner -encoders | grep nvenc

echo -e "\nSearching for NVIDIA Libraries:"
find /usr -name "libnvidia-encode.so*" 2>/dev/null || echo "No encoder libraries found"

echo -e "\nChecking Container GPU Access:"
ls -la /dev/nvidia* 2>/dev/null || echo "No NVIDIA devices in container"

echo -e "\nChecking Dynamic Library Path:"
echo $LD_LIBRARY_PATH

echo -e "\nChecking if host NVIDIA libraries are mounted in container:"
find /usr/lib/x86_64-linux-gnu -name "libnvidia*" | sort