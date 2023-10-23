import traceback as tb
import time
import rclpy
import asyncio
import tornado
import tornado.ioloop
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from bob_shared.node_runner import NodeRunner
from bob_interfaces.msg import Tracking, TrackingState

from .metrics_server import MetricsServer
from .handlers import PrometheusMetricsHandler

class PrometheusNode(object):
  
  instance = None

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile, node_name = "bob_prometheus"):
    self.__class__.instance = self

    self.node = rclpy.create_node(node_name)

    self.logger = self.node._logger

    self.sub_status = self.node.create_subscription(Tracking, 'bob/tracker/tracking', self.state_callback, subscriber_qos_profile)
    self.pub_tracking_state_json = self.node.create_publisher(String, 'bob/tracker/tracking/json', publisher_qos_profile)

    self.node.declare_parameters(namespace='', parameters=[('port', 8082)])
    self.metrics_server_port = self.node.get_parameter('port').value

    self.logger.info(f'bob_prometheus --> metrics server ports {self.metrics_server_port}.')

    self.metrics_server = MetricsServer(port=self.metrics_server_port)

    # tornado event loop stuff
    self.event_loop = None
    asyncio.set_event_loop(asyncio.new_event_loop())
    self.event_loop = tornado.ioloop.IOLoop()
    self.metrics_server.start()

    # tornado event loop. all the web server and web socket stuff happens here
    threading.Thread(target = self.event_loop.start, daemon = True).start()
    
    self.logger.info(f'{node_name} node is up and running.')

  def start(self):
    rclpy.spin(self.node)

  def destroy_node(self):
    self.node.destroy_node()

  def state_callback(self, msg_tracking:Tracking):
    PrometheusMetricsHandler.state = msg_tracking.state
    string_msg = String()
    string_msg.data = f"{{\"trackable\":{msg_tracking.state.trackable}, \"alive\":{msg_tracking.state.alive}, \"started\":{msg_tracking.state.started}, \"ended\":{msg_tracking.state.ended}}}"
    self.pub_tracking_state_json.publish(string_msg)    

def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = QoSProfile(depth=1)
  subscriber_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  subscriber_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  subscriber_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  publisher_qos_profile = QoSProfile(depth=1)
  publisher_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  publisher_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  publisher_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  node = PrometheusNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node.node)
  runner.run()

if __name__ == '__main__':
  main()