import math
import time
## until abosolutemove is taken from onvif
from onvif2 import ONVIFCamera
##

#For testing Purposes
Rasterstep = 0
completeMSG = False 
initiate_v1 = True

while(completeMSG == False):
    if( (initiate_v1 == True)  or  (RasterImageACK_v1 == True)):
        #initially magic numbers, later ros-msg-values:
        startX = -1; endX = 1; startY = -1; endY = 1
        stepwidthX = 0.4; stepwidthY = 0.4
        XIncrementsPerY = math.ceil(abs(startX-endX)/stepwidthX)+1
        YStepsTotal = math.ceil(abs(startY-endY)/stepwidthY)
        if(Rasterstep <= XIncrementsPerY*YStepsTotal):
            currentStepX = round(startX + (stepwidthX*(Rasterstep % XIncrementsPerY)),10)
            currentStepY = round(startY + (stepwidthY*(math.floor(Rasterstep  / XIncrementsPerY))),10)
            
            ############### Remove later and use ros2-AbsoluteMove message instead #############
                            
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

            XMAX = 1
            XMIN = -1
            YMAX = 1
            YMIN = -1
            ZoomMAX = 1
            ZoomMIN = 0

            moverequest.Position.PanTilt.x = min(max(currentStepX,XMIN),XMAX)
            moverequest.Position.PanTilt.y = min(max(currentStepY,YMIN),YMAX)
            moverequest.Position.Zoom.x = min(max(0,ZoomMIN),ZoomMAX)


            #global active
            if active:
                ptz.Stop({'ProfileToken': moverequest.ProfileToken})
            active = True  
            ptz.AbsoluteMove(moverequest)
  



            ####################################################################################
            
            
            #Polling ONVIF if target has been reached
            #while((ONVIFGetprofile.X != currentStepX) and (ONVIFGetprofile.Y != currentStepY)):
                #time.sleep(0.1)
            time.sleep(4.0)

            print(f"Ros2 MSG calibrate/v1, CurrentStepX: {currentStepX}, CurrentStepY: {currentStepY}")
        else:
            completeMSG = True
            print("ROS2 MSG bob/ptz/calibrate/v1/complete.msg ACK")
    
    #For testing purposes:
    initiate_v1 = False
    RasterImageACK_v1 = False
    time.sleep(0.2)
    RasterImageACK_v1 = True
    if completeMSG == False:
        print(f"MSG RasterImageACK_v1 = {RasterImageACK_v1}")
    Rasterstep += 1