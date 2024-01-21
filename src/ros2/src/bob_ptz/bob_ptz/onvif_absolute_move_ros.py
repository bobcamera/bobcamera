import asyncio, sys
from onvif2 import ONVIFCamera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import traceback as tb
import rclpy
from typing import List
from bob_interfaces.msg import PTZAbsoluteMove
from bob_shared.node_runner import NodeRunner
IP="10.20.30.140"   # Camera IP address
PORT=80           # Port
USER="bob"         # Username
PASS="Sky360Sky!"        # Password


XMAX = 1
XMIN = -1
YMAX = 1
YMIN = -1
moverequest = None
ptz = None
active = False

def setup_move():
    mycam = ONVIFCamera(IP, PORT, USER, PASS, '/workspaces/bobcamera/src/ros2/src/bob_monitor/resource/wsdl')
    # Create media service object
    media = mycam.create_media_service()
    
    # Create ptz service object
    global ptz
    ptz = mycam.create_ptz_service()

    # Get target profile
    media_profile = media.GetProfiles()[0]

    # Get PTZ configuration options for getting continuous move range
    request = ptz.create_type('GetConfigurationOptions')
    request.ConfigurationToken = media_profile.PTZConfiguration.token
    ptz_configuration_options = ptz.GetConfigurationOptions(request)

    global moverequest
    moverequest = ptz.create_type('AbsoluteMove')
    moverequest.ProfileToken = media_profile.token
    if moverequest.Position is None:
        moverequest.Position = ptz.GetStatus({'ProfileToken': media_profile.token}).Position


    # Get range of pan and tilt
    # NOTE: X and Y are Position vector
    global XMAX, XMIN, YMAX, YMIN
    XMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Max
    XMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].XRange.Min
    YMAX = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Max
    YMIN = ptz_configuration_options.Spaces.ContinuousPanTiltVelocitySpace[0].YRange.Min
    print(f"XMAX={XMAX}")
    print(f"XMIN={XMIN}")
    print(f"YMAX={YMAX}")
    print(f"YMIN={YMIN}")



def OnvifAbsoluteMoveFromRosMsg():
    """Reading from stdin and displaying menu"""
    global moverequest, ptz
    
    posx = sys.stdin.readline().strip("\n")
    posy = sys.stdin.readline().strip("\n")
    moverequest.Position.PanTilt.x = posx
    moverequest.Position.PanTilt.y = posy
    global active
    if active:
        ptz.Stop({'ProfileToken': moverequest.ProfileToken})
    active = True  
    ptz.AbsoluteMove(moverequest)

class AbsoluteMoveNode(Node):

    def __init__(self, subscriber_qos_profile: QoSProfile, publisher_qos_profile: QoSProfile):
        super().__init__('bob_cloud_estimator')



        self.pub_environment_data = self.create_publisher(PTZAbsoluteMove, 'bob/ptz/move/absolute', publisher_qos_profile)


        self.declare_parameters(namespace='',
                            parameters=[('observer_timer_interval', 30)])

        # setup services, publishers and subscribers    
        self.sub_PTZPosition = self.create_subscription(PTZAbsoluteMove, 'bob/ptz/move/absolute', self.OnvifAbsoluteMoveFromRosMsg, subscriber_qos_profile)
        self.get_logger().info(f'{self.get_name()} node is up and running.')

        IP="10.20.30.140"   # Camera IP address
        PORT=80           # Port
        USER="bob"         # Username
        PASS="Sky360Sky!"        # Password


        XMAX = 1
        XMIN = -1
        YMAX = 1
        YMIN = -1
        moverequest = None
        ptz = None
        active = False

        def setup_move():
                mycam = ONVIFCamera(IP, PORT, USER, PASS, '/workspaces/bobcamera/src/ros2/src/bob_monitor/resource/wsdl')
                # Create media service object
                media = mycam.create_media_service()
                
                # Create ptz service object
                global ptz
                ptz = mycam.create_ptz_service()

                # Get target profile
                media_profile = media.GetProfiles()[0]

                # Get PTZ configuration options for getting continuous move range
                request = ptz.create_type('GetConfigurationOptions')
                request.ConfigurationToken = media_profile.PTZConfiguration.token
                ptz_configuration_options = ptz.GetConfigurationOptions(request)

                global moverequest
                moverequest = ptz.create_type('AbsoluteMove')
                moverequest.ProfileToken = media_profile.token
                if moverequest.Position is None:
                    moverequest.Position = ptz.GetStatus({'ProfileToken': media_profile.token}).Position
        
        setup_move()


    def OnvifAbsoluteMoveFromRosMsg(self, msg_position):
            """Reading from stdin and displaying menu"""
            global moverequest, ptz
            self.msg_position = msg_position
            moverequest.Position.PanTilt.x = self.msg_position.pospantiltx
            moverequest.Position.PanTilt.y = self.msg_position.pospantilty
            global active
            if active:
                ptz.Stop({'ProfileToken': moverequest.ProfileToken})
            active = True  
            ptz.AbsoluteMove(moverequest)









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