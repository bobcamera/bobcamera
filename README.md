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

[Acknowledgements, credits and references](REFERENCES.md)