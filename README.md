# BOB the Universal Object Tracker

Below are the installation instructions for installing BOB, your friendly Object (bird, insect, bat, uap) detector, tracker and recorder
The application uses Docker and Docker Compose to run, at present we are still in a very early release phase so this the install process is not polished at all. We will endeavour to improve this over time to please bear with us.

*It is possible to run Bobcamera on Windows 10 or Windows 11 through WSL. Please refer to the [Windows setup instructions](WINDOWS_SETUP.md) to prep your machine before running the steps below.*

## The following steps will need to be performed in a linux terminal

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
```
./run.sh
```
### 8. Use your system browser and navigate to [http://localhost:8080](http://localhost:8080)

### 9. To shut BOB down, type CTRL + C in the terminal

---

To change configuration please execute: 
```
./config.sh 
```

To reset to default configuration, please execute: 
```
./setup.sh 
```
