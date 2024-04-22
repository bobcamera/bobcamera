import asyncio
import websockets
import picamera
import io
async def image_stream(websocket, path):
    with picamera.PiCamera() as camera:
        camera.resolution = (1280, 720)  # Set resolution
        stream = io.BytesIO()
            for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            await websocket.send(stream.read())
            stream.seek(0)
            stream.truncate()
start_server = websockets.serve(image_stream, '0.0.0.0', 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()