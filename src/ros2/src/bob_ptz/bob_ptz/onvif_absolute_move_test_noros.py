import asyncio, sys
from onvif2 import ONVIFCamera
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import traceback as tb
import rclpy
from typing import List
#ro2 topic pub -1 /bob/ptz/absolute/move bob_interfaces/msg/PTZAbsoluteMove "{pospantiltx: -0.25, pospantilty: 1, poszoomx: 0, speedpantiltx: 1, speedpantilty: 1, speedzoomx: 1}"
IP="10.20.30.140"   # Camera IP address
PORT=80           # Port
USER="bob"         # Username
PASS="Sky360Sky!"        # Password
# Get range of pan and tilt
# NOTE: X and Y are Position vector
global XMAX, XMIN, YMAX, YMIN
XMAX = 1
XMIN = -1
YMAX = 1
YMIN = -1
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

def OnvifAbsoluteMoveFromRosMsg():
        global moverequest, ptz
 
        moverequest.Position.PanTilt.x = 0.5
        moverequest.Position.PanTilt.y = -1
        global active
        if active:
            ptz.Stop({'ProfileToken': moverequest.ProfileToken})
        active = True  
        ptz.AbsoluteMove(moverequest)
OnvifAbsoluteMoveFromRosMsg();
# Spin to allow the message to be sent
#rclpy.spin_once(Node)
