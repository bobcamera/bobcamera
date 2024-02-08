import asyncio, sys
from onvif2 import ONVIFCamera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import traceback as tb
import rclpy
from typing import List

from bob_interfaces.msg import PTZAbsoluteMove
from bob_shared.node_runner import NodeRunner

#execute in command line: 
#colcon build --packages-select bob_interfaces
#source install/setup.bash
#ros2 topic info /bob/ptz/move/absolute
#ros2 interface show bob_interfaces/msg/PTZAbsoluteMove
#rqt
#ros2 topic pub -1 /bob/ptz/move/absolute bob_interfaces/msg/PTZAbsoluteMove "{pospantiltx: -0.25, pospantilty: 1, poszoomx: 0, speedpantiltx: 1, speedpantilty: 1, speedzoomx: 1}"
#cmd ptz GetStatus  {'ProfileToken': 'Profile_3'}

class AbsoluteMoveNode(Node):

    def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('onvif_absolute_move_ros')



        self.pub_environment_data = self.create_publisher(PTZAbsoluteMove, 'bob/ptz/move/absolute', publisher_qos_profile)


        self.declare_parameters(namespace='',
                            parameters=[('observer_timer_interval', 30)])

        # setup services, publishers and subscribers    
        self.sub_PTZPosition = self.create_subscription(PTZAbsoluteMove, 'bob/ptz/move/absolute', self.AbsoluteMoveFromRosMsg, subscriber_qos_profile)
        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def AbsoluteMoveFromRosMsg(self, msg_position):
            """Reading from stdin and displaying menu"""
            self.get_logger().info("Received PTZAbsoluteMove message.")

            IP="10.20.30.140"   # Camera IP address
            PORT=80           # Port
            USER="bob"         # Username
            PASS="Sky360Sky!"        # Password
        # Get range of pan and tilt

            XMAX = 1
            XMIN = -1
            YMAX = 1
            YMIN = -1
            ZoomMAX = 1
            ZoomMIN = 0
            active = False

            mycam = ONVIFCamera(IP, PORT, USER, PASS, '/workspaces/bobcamera/src/ros2/src/bob_monitor/resource/wsdl')
            # Create media service object
            media = mycam.create_media_service()
            
            # Create ptz service object
            ptz = mycam.create_ptz_service()

            # Get target profile
            media_profile = media.GetProfiles()[0]

            moverequest = ptz.create_type('AbsoluteMove')
            moverequest.ProfileToken = media_profile.token
            if moverequest.Position is None:
                moverequest.Position = ptz.GetStatus({'ProfileToken': media_profile.token}).Position
            


            #global moverequest, ptz
            self.msg_position = msg_position

            XMAX = 1
            XMIN = -1
            YMAX = 1
            YMIN = -1
            ZoomMAX = 1
            ZoomMIN = 0

            #moverequest.Position.PanTilt.x = min(max(self.msg_position.pospantiltx,XMIN),XMAX)
            #moverequest.Position.PanTilt.y = min(max(self.msg_position.pospantilty,YMIN),YMAX)
            #moverequest.Position.Zoom.x = min(max(self.msg_position.poszoomx,ZoomMIN),ZoomMAX)

            moverequest.Position.PanTilt.x = 0.0
            moverequest.Position.PanTilt.y = 0.0
            moverequest.Position.Zoom.x = min(max(self.msg_position.poszoomx,ZoomMIN),ZoomMAX)

            #global active
            if active:
                ptz.Stop({'ProfileToken': moverequest.ProfileToken})
            active = True  
            ptz.AbsoluteMove(moverequest)
  

    # Spin to allow the message to be sent
    #rclpy.spin_once(Node)








def main(args=None):

  rclpy.init(args=args)

  subscriber_qos_profile = QoSProfile(depth=10)
  subscriber_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  subscriber_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  subscriber_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  publisher_qos_profile = QoSProfile(depth=10)
  publisher_qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
  publisher_qos_profile.durability = QoSDurabilityPolicy.VOLATILE
  publisher_qos_profile.history = QoSHistoryPolicy.KEEP_LAST

  node = AbsoluteMoveNode(subscriber_qos_profile, publisher_qos_profile)

  runner = NodeRunner(node)
  runner.run()

if __name__ == '__main__':
  main()