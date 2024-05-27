# Define Variables
height = 720           # Height of the captured image
width = 1280           # Width of the captured image
port = 8765            # Port number on which the WebSocket server will listen
host = '0.0.0.0'       # Host IP address for the WebSocket server. '0.0.0.0' allows connections from any IP address
image_format = 'jpeg'  # Format of the captured images
use_video_port = True  # Set to True for faster capture rate with lower latency, False for higher quality still images

# Import Libraries

# Import the asyncio library to run the websocket server
import asyncio
# Import the websockets library to create the websocket server
import websockets
# Import the picamera library to capture images from the camera
import picamera
# Import the io library to stream images to the connected client
import io

# Define the image stream function
async def image_stream(websocket, path):
    # Create a new camera object
    with picamera.PiCamera() as camera:
        # Set the camera resolution
        camera.resolution = (width, height)
        # Start the camera preview
        stream = io.BytesIO()
        # Continuously capture images from the camera
        for _ in camera.capture_continuous(stream, image_format, use_video_port=use_video_port):
            # Reset the stream position to the beginning
            stream.seek(0)
            # Send the image to the connected client
            await websocket.send(stream.read())
            # Reset the stream for the next capture
            stream.seek(0)
            # Truncate the stream to clear the image data
            stream.truncate()

# Start the websocket server
start_server = websockets.serve(image_stream, host, port)

# Run the server until it is stopped
asyncio.get_event_loop().run_until_complete(start_server)
# Run the server forever
asyncio.get_event_loop().run_forever()