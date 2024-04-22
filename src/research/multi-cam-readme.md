This code was used to stream images from two Raspberry Pis each with a Raspberry Pi HQ 12.3MP Camera (1280x720 JPEG images).
The python server script accesses the camera and streams the images as JPEGs via websocket connection.
On the 3rd computer (Mac) an HTML file viewed in Chrome was used to view the 2 image feeds from the Raspberry Pi Cameras combined and positioned in 3D space. 
The latency was pretty low and I'll be testing other parameters, larger images, more feeds, etc.
Use Picamera NOT Picamera2 on the Raspberry Pi Server Scripts.
The web socket ports must be different on each Pi. 
I used port 8765 on one and 8766 on the other. 
The correct websocket URLs (the IP address of the Raspberry Pi) with the same ports used in both pi-image-server.py files will need to be entered in the HTML file as well. 

launch the server script on each Pi using the following terminal command:

python pi-image-server.py 

Edit the HTML file so it has the correct websocket URLs with ports. 
Then open the html file on the client device in a web browser to view the two feeds.
