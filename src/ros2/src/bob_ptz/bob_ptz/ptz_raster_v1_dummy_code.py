import math
import time



Rasterstep = 0
completeMSG = False 
initiate_v1 = True

startX = 0.7
endX =  0.8
stepwidthX = 0.1
startY = 0.6
endY = 1
stepwidthY = 0.2
startZoom = 0.0
endZoom = 0.1
stepwidthZoom = 0.05
while(completeMSG == False):
    if( (initiate_v1 == True)  or  (RasterImageACK_v1 == True)):
        #initially magic numbers, later ros-msg-values:
        YIncrementsPerX = math.ceil(abs(startY-endY)/stepwidthY)+1
        XIncrementsPerZoommultiplesOfY = math.ceil(YIncrementsPerX*(1+(abs(startX-endX)/stepwidthX)))
        ZoomStepsTotal = math.ceil(abs(startZoom-endZoom)/stepwidthZoom)+1
                
        if(Rasterstep < XIncrementsPerZoommultiplesOfY*ZoomStepsTotal):
            currentStepY = round(startY+(stepwidthY*(Rasterstep % YIncrementsPerX )),10)
            currentStepX = round(startX + (stepwidthX*math.floor((Rasterstep % XIncrementsPerZoommultiplesOfY)/ YIncrementsPerX )) ,10)
            currentStepZoom = round(startZoom + (stepwidthZoom*(math.floor(Rasterstep  / XIncrementsPerZoommultiplesOfY ))),10)




            #Polling ONVIF if target has been reached
            #while((ONVIFGetprofile.X != currentStepX) and (ONVIFGetprofile.Y != currentStepY)):
                #time.sleep(0.1)
            print(f"Ros2 MSG calibrate/v1, CurrentStepX: {currentStepX}, CurrentStepY: {currentStepY}, CurrentStepZoom: {currentStepZoom}")
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