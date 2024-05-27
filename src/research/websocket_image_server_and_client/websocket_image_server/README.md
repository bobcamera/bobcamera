# Raspberry Pi Camera Stream Server

This project sets up a WebSocket server on a Raspberry Pi to stream images captured by the Raspberry Pi camera. The server captures images using the `picamera` library and streams them to connected clients.

## Prerequisites

- Raspberry Pi with Raspberry Pi OS
- Raspberry Pi Camera Module
- Internet connection

## Installation

Follow these steps to install the necessary dependencies and enable the camera interface.

### Step 1: Download the Installation Script

Download the `install_dependencies.sh` script to your Raspberry Pi.

### Step 2: Make the Script Executable

Open a terminal and navigate to the directory where you downloaded the script. Make the script executable by running:

```bash
chmod +x install_dependencies.sh
