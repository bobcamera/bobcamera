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
#ros2 topic info /bob/ptz/absolute/move
#ros2 interface show bob_interfaces/msg/PTZAbsoluteMove
#rqt
#ros2 topic pub -1 /bob/ptz/absolute/move bob_interfaces/msg/PTZAbsoluteMove "{pospantiltx: -0.25, pospantilty: 1, poszoomx: 0, speedpantiltx: 1, speedpantilty: 1, speedzoomx: 1}"


class AbsoluteMoveNode(Node):

    def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('bob_cloud_estimator')



        self.pub_environment_data = self.create_publisher(PTZAbsoluteMove, 'bob/ptz/move/absolute', publisher_qos_profile)


        self.declare_parameters(namespace='',
                            parameters=[('observer_timer_interval', 30)])

        # setup services, publishers and subscribers    
        self.sub_PTZPosition = self.create_subscription(PTZAbsoluteMove, 'bob/ptz/move/absolute', self.OnvifAbsoluteMoveFromRosMsg, subscriber_qos_profile)
        self.get_logger().info(f'{self.get_name()} node is up and running.')

    def OnvifAbsoluteMoveFromRosMsg(self, msg_position):
            """Reading from stdin and displaying menu"""
                
            IP="10.20.30.140"   # Camera IP address
            PORT=80           # Port
            USER="bob"         # Username
            PASS="Sky360Sky!"        # Password
        # Get range of pan and tilt
            # NOTE: X and Y are Position vector
            global XMAX, XMIN, YMAX, YMIN, ZoomMAX, ZoomMIN

            XMAX = 1
            XMIN = -1
            YMAX = 1
            YMIN = -1
            ZoomMAX = 1
            ZoomMIN = 0
            moverequest = None
            ptz = None
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

            moverequest.Position.PanTilt.x = min(max(self.msg_position.pospantiltx,XMIN),XMAX)
            moverequest.Position.PanTilt.y = min(max(self.msg_position.pospantilty,YMIN),YMAX)
            moverequest.Position.Zoom.x = min(max(self.msg_position.poszoomx,ZoomMIN),ZoomMAX)


            '''
            if XMAX <= self.msg_positiontiltx:
                moverequest.Position.PanTilt.x = XMAX
            elif self.msg_positiontiltx <= XMIN:
                moverequest.Position.PanTilt.x = XMIN
            else:
                moverequest.Position.PanTilt.x = self.msg_position.pospantiltx
        
            if YMAX <= self.msg_position.pospantilty:
                moverequest.Position.PanTilt.y = YMAX
            elif self.msg_position.pospantilty <= XMIN:
                moverequest.Position.PanTilt.y = YMIN
            else:
                moverequest.Position.PanTilt.y = self.msg_position.pospantilty

            if ZoomMAX <= self.msg_position.poszoomx:
                moverequest.Position.Zoom.x = ZoomMAX
            elif self.msg_position.poszoomx <= ZoomMIN:
                moverequest.Position.Zoom.x = ZoomMIN
            else:
                moverequest.Position.Zoom.x = self.msg_position.poszoomx
            '''
            #moverequest.Position.PanTilt.x = self.msg_position.pospantiltx
            #moverequest.Position.PanTilt.y = self.msg_position.pospantilty
            #moverequest.Position.Zoom.x = self.msg_position.poszoomx

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