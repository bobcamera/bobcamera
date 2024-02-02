import math
import time






#For testing Purposes
Rasterstep = 0
completeMSG = False 
initiate_v1 = True

'''
	Start X: -1
	Start Y: -1
	Start Zoom: 0.0
	End X: -0.6
	End Y: -0.4
	End Zoom: 2.0
	stepwidthX: 0.2
	stepwidthY: 0.2
	stepwidthZoom: 1
'''
while(completeMSG == False):
    if( (initiate_v1 == True)  or  (RasterImageACK_v1 == True)):
        #initially magic numbers, later ros-msg-values:
        startX = -1; endX = -0.6; startY = -1; endY = -0.4; startZoom = 0; endZoom = 1
        stepwidthX = 0.2; stepwidthY = 0.2; stepwidthZoom = 0.2
        ZoomIncrementsPerX = math.ceil(abs(startZoom-endZoom)/stepwidthZoom)+1
        XIncrementsPerYmultiplesOfZoom = math.ceil(ZoomIncrementsPerX*(abs(startX-endX)/stepwidthX)+1)
        YStepsTotal = math.ceil(abs(startY-endY)/stepwidthY)+1
        currentZoom = startZoom
        if(Rasterstep < XIncrementsPerYmultiplesOfZoom*YStepsTotal):
            currentZoom = round(startZoom+(stepwidthZoom*(Rasterstep % ZoomIncrementsPerX )),10)
            currentStepX = round(startX + (stepwidthX*(Rasterstep % (XIncrementsPerYmultiplesOfZoom % ZoomIncrementsPerX) )),10)
            currentStepY = round(startY + (stepwidthY*(math.floor(Rasterstep  / XIncrementsPerYmultiplesOfZoom / ZoomIncrementsPerX))),10)
            #Polling ONVIF if target has been reached
            #while((ONVIFGetprofile.X != currentStepX) and (ONVIFGetprofile.Y != currentStepY)):
                #time.sleep(0.1)
            print(f"Ros2 MSG calibrate/v1, CurrentStepX: {currentStepX}, CurrentStepY: {currentStepY}, CurrentZoom: {currentZoom}")
        else:
            completeMSG = True
            print("ROS2 MSG bob/ptz/calibrate/v1/complete.msg ACK")
    
    #For testing purposes:
    initiate_v1 = False
    RasterImageACK_v1 = False
    #time.sleep(1.0)
    RasterImageACK_v1 = True
    if completeMSG == False:
        print(f"MSG RasterImageACK_v1 = {RasterImageACK_v1}")
    Rasterstep += 1