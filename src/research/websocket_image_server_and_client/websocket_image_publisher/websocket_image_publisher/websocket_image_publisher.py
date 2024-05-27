import asyncio
import websockets
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import rclpy

class WebSocketImagePublisher(Node):
    def __init__(self, topic_name, websocket_url, websocket_port, fps, width, height):
        super().__init__('websocket_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, topic_name, 10)
        self.websocket_uri = f'ws://{websocket_url}:{websocket_port}'
        self.fps = fps
        self.width = width
        self.height = height
        self.reconnect_interval = 1  # Reduced time in seconds before trying to reconnect
        self.get_logger().info(f'Connecting to WebSocket server at {self.websocket_uri}')
        asyncio.get_event_loop().run_until_complete(self.connect_to_websocket())

    async def connect_to_websocket(self):
        while True:
            try:
                async with websockets.connect(
                    self.websocket_uri,
                    ping_interval=20,  # Increased ping interval
                    ping_timeout=60    # Increased ping timeout
                ) as websocket:
                    self.websocket = websocket
                    self.get_logger().info(f'Successfully connected to {self.websocket_uri}')
                    await self.receive_images()
            except asyncio.TimeoutError:
                self.get_logger().error(f'Timeout while connecting to {self.websocket_uri}')
            except websockets.ConnectionClosed as e:
                self.get_logger().info(f'WebSocket connection closed: {e}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to {self.websocket_uri}: {e}')
            self.get_logger().info(f'Retrying connection in {self.reconnect_interval} seconds...')
            await asyncio.sleep(self.reconnect_interval)

    async def receive_images(self):
        try:
            while True:
                message = await asyncio.wait_for(self.websocket.recv(), timeout=60)  # Increased timeout
                self.process_image(message)
                await asyncio.sleep(1 / self.fps)
        except asyncio.TimeoutError:
            self.get_logger().error(f'Timed out waiting for image from {self.websocket_uri}')
        except websockets.ConnectionClosed:
            self.get_logger().info(f'WebSocket connection closed')
        except Exception as e:
            self.get_logger().error(f'Error receiving images: {e}')

    def process_image(self, data):
        try:
            # Convert the binary data to a numpy array
            np_arr = np.frombuffer(data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Resize the image to the specified width and height
            if self.width > 0 and self.height > 0:
                image_np = cv2.resize(image_np, (self.width, self.height))

            # Convert the OpenCV image to a ROS2 CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np_arr.tobytes()

            # Log header information and verify the format and data size
            # self.get_logger().info(f'Publishing image with header: {msg.header}')
            # self.get_logger().info(f'Image format: {msg.format}, Data size: {len(msg.data)} bytes')

            # Publish the message
            self.publisher_.publish(msg)
            # self.get_logger().info('Published an image.')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Parameters
    topic_name = '/bob/frames/annotated/resized/compressed'
    # websocket_url = '192.168.1.166'
    # websocket_port = 8767
    websocket_url = '192.168.1.213'
    websocket_port = 8766
    fps = 20
    width = 1920
    height = 1080
    # width = 4056
    # height = 3040

    node = WebSocketImagePublisher(topic_name, websocket_url, websocket_port, fps, width, height)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()