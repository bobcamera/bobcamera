import rclpy
from rclpy.node import Node
from bob_interfaces.srv import ImageRaster

class RasterPTZClient(Node):
    def __init__(self):
        super().__init__('raster_ptz_client')

        # Create a client for the ImageRaster service
        self.client = self.create_client(ImageRaster, '/image_acquisition')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ImageRaster service not available, waiting...')
        
        # Create a request object
        self.request = ImageRaster.Request()

        # Set the values of the request
        self.request.x = 1  # Replace with the actual values
        self.request.y = 2
        self.request.zoom = 3.0
        self.request.campaign = 'example_campaign'

        # Call the service
        self.call_service()

    def call_service(self):
        # Call the service with the request
        future = self.client.call_async(self.request)

        # Wait for the service to complete
        rclpy.spin_until_future_complete(self, future)

        # Process the response
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Service call result: {response.success}")
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = RasterPTZClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()