# BOB the Universal Object Tracker

## Please Note: These instructions will install the latest development release, we would highly recommend you rather install the latest stable release which can be found [here](https://github.com/bobcamera/bobinstall).

Below are the installation instructions for installing BOB, your friendly Object (bird, insect, bat, uap) detector, tracker and recorder
The application uses Docker and Docker Compose to run, at present we are still in a very early release phase so this the install process is not polished at all. We will endeavour to improve this over time so please bear with us.

*It is possible to run Bobcamera on Windows 10 or Windows 11 through WSL. Please refer to the [Windows setup instructions](WINDOWS_SETUP.md) to prep your machine before running the steps below.*

## The following steps will need to be performed in a linux terminal

- Ubuntu 24.04 has been tested
- Ubuntu 23.10 has been tested
- Ubuntu 22.04 has been tested

### 1. Install dependencies
- Git
- Curl
```
sudo apt-get install git curl
```
### 2. Clone the github repo
```
git clone --recursive https://github.com/bobcamera/bobcamera.git
```
### 3. Change directory to bobcamera
```
cd bobcamera
```
### 4. Run the setup script, this will install docker and docker compose
```
./setup.sh
```
### 5. Reboot the machine
- Only required if Docker has been installed during the setup script
```
sudo shutdown -r now
```
### 6. Navigate back to the bobcamera directory once rebooted
```
cd ~/bobcamera
```
### 7. To start up BOB

#### First-time
##### Update RTSP cam details if you are using an RTSP camera

* copy the ex_config_rtsp.yaml to a file of your on: 
```
cp ex_config_rtsp.yaml my_rtsp_config.yaml
```
* open the yaml file with your prefeered text editor.
```
code my_rtsp_config.yaml
```
* update the camera uri in your yaml file in the camera node: `rtsp_uri: 'YOUR_RTSP_URI'`
* set the source variable to be `source_type: 'RTSP_STREAM'`

##### Update USB cam details if you are using a USB camera

* copy the ex_config_rtsp.yaml to a file of your on: 
```
cp ex_config_rtsp.yaml my_usb_config.yaml
```
* open the yaml file with your prefeered text editor.
```
code my_usb_config.yaml
```
* update the variables for the camera id in your yaml file: `camera_id: 0`
* set the BOB_SOURCE environment variable to be `source_type: 'USB_CAMERA'`

#### To run
```
./run.sh <NAME_OF_YOUR_YAML_FILE>
```

### 8. Use your system browser and navigate to [http://localhost:8080](http://localhost:8080)

### 9. To shut BOB down, type CTRL + C in the terminal

---

## 🚀 UI Quick Start

BOB includes a modern **React 19 + Mantine UI** with real-time streaming and system monitoring.

### Build the UI Locally

```bash
cd ui
npm ci                 # Install dependencies
npm run build          # Build for production
npm run dev            # Start dev server (http://localhost:5173)
```

### Build & Run Docker Image

```bash
# Quick build (includes git in image)
docker build -t bobcamera/bob-ui:latest ui/

# Build with git commit info (recommended)
GIT_COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
docker build --build-arg GIT_COMMIT=$GIT_COMMIT -t bobcamera/bob-ui:latest ui/

# Run the container
docker run -d -p 8080:80 bobcamera/bob-ui:latest
```

### Using the Makefile (Recommended)

```bash
cd ui
make help              # Show all targets
make docker-build      # Auto-detects git commit
make docker-run        # Run on port 8080
```

### Configuration

Copy and customize environment:

```bash
cp ui/.env.example ui/.env
# Edit ui/.env to change API URLs, feature flags, etc.
```

### Full Documentation

For complete setup, troubleshooting, and development guide:
→ **[UI README](ui/README.md)** - Full documentation
→ **[.env Configuration](ui/.env.example)** - All environment variables

### Key Features

- 🎯 Real-time video streams with detection overlays
- 📊 System metrics (CPU, GPU, memory, disk)
- 📹 Camera management and recording playback
- 📝 Live log streaming
- ⚙️ Configuration management
- 🌙 Dark/light theme toggle
- ♿ WCAG 2.1 compliant

---

--- 
## Appendix: 

### I. To change the configuration: 
 * Edit the YAML file you use.

### II. To update to the lastest version of bob please execute: 
```
git pull origin main
```

### III. Reset the config file to factory conditions: 
```
cp ex_config.yaml my_config_file.yaml
```

-----

## UI Development

BOB Camera includes a modern React frontend built with Mantine UI v8. The UI provides:

- **Real-time monitoring** - Live camera feeds, detections, and system metrics
- **Camera management** - Configure RTSP/USB cameras, test connections
- **Track history** - Browse and filter detection events
- **Recordings** - View and download saved clips
- **System health** - Monitor CPU, GPU, memory, and services
- **Dark/Light mode** - Comfortable viewing in any environment

### Quick Start (UI Development)

```bash
cd ui
npm install
npm run dev
```

Open [http://localhost:5173](http://localhost:5173) - the UI will proxy to the backend on port 8080.

**Documentation:**
- [Quick Start Guide](ui/QUICKSTART_MANTINE.md) - Get started in 5 minutes
- [Full Documentation](ui/MANTINE_SETUP.md) - Complete setup and development guide

### UI Tech Stack

- **React 19** + TypeScript + Vite
- **Mantine UI v8** - Modern component library
- **TailwindCSS v4** - Utility-first CSS (layout only)
- **Zustand** - State management
- **React Router v7** - Client-side routing
- **Axios** - REST API client
- **WebSocket** - Real-time updates

-----

[Acknowledgements, credits and references](REFERENCES.md)