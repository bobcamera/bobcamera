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
from bob_interfaces.msg import TrackingState

from .metrics_server import MetricsServer
from .handlers import PrometheusMetricsHandler

class PrometheusNode(object):
  
  instance = None

  def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile, node_name = "bob_prometheus"):
    self.__class__.instance = self

    self.node = rclpy.create_node(node_name)

    self.sub_status = self.node.create_subscription(TrackingState, 'bob/tracker/tracking_state', self.state_callback, subscriber_qos_profile)
    self.pub_tracking_state_json = self.node.create_publisher(String, 'bob/tracker/tracking_state/json', publisher_qos_profile)

    self.metrics_server = MetricsServer(8099)

    # tornado event loop stuff
    self.event_loop = None
    asyncio.set_event_loop(asyncio.new_event_loop())
    self.event_loop = tornado.ioloop.IOLoop()
    self.metrics_server.start()

    # tornado event loop. all the web server and web socket stuff happens here
    threading.Thread(target = self.event_loop.start, daemon = True).start()

    self.logger = self.node._logger
    self.logger.info(f'{node_name} node is up and running.')

  def start(self):
    rclpy.spin(self.node)

  def destroy_node(self):
    self.node.destroy_node()

  def state_callback(self, msg_tracking_state:TrackingState):
    PrometheusMetricsHandler.state = msg_tracking_state
    string_msg = String()
    string_msg.data = f"{{\"trackable\":{msg_tracking_state.trackable}, \"alive\":{msg_tracking_state.alive}, \"started\":{msg_tracking_state.started}, \"ended\":{msg_tracking_state.ended}}}"
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