import math
import time


#For testing Purposes
Rasterstep = 0
completeMSG = False 
initiate_v1 = True

while(completeMSG == False):
    if( (initiate_v1 == True)  or  (RasterImageACK_v1 == True)):
        #initially magic numbers, later ros-msg-values:
        startX = -1; endX = 1; startY = -1; endY = 1
        stepwidthX = 0.2; stepwidthY = 0.2
        XIncrementsPerY = math.ceil(abs(startX-endX)/stepwidthX)+1
        YStepsTotal = math.ceil(abs(startY-endY)/stepwidthY)
        if(Rasterstep <= XIncrementsPerY*YStepsTotal):
            currentStepX = round(startX + (stepwidthX*(Rasterstep % XIncrementsPerY)),10)
            currentStepY = round(startY + (stepwidthY*(math.floor(Rasterstep  / XIncrementsPerY))),10)
            #Polling ONVIF if target has been reached
            #while((ONVIFGetprofile.X != currentStepX) and (ONVIFGetprofile.Y != currentStepY)):
                #time.sleep(0.1)
            print(f"Ros2 MSG calibrate/v1, CurrentStepX: {currentStepX}, CurrentStepY: {currentStepY}")
        else:
            completeMSG = True
            print("ROS2 MSG bob/ptz/calibrate/v1/complete.msg ACK")
    
    #For testing purposes:
    initiate_v1 = False
    RasterImageACK_v1 = False
    time.sleep(1.0)
    RasterImageACK_v1 = True
    if completeMSG == False:
        print(f"MSG RasterImageACK_v1 = {RasterImageACK_v1}")
    Rasterstep += 1