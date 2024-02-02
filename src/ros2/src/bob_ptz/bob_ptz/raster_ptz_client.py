import rclpy
from rclpy.node import Node
from onvif2 import ONVIFCamera
from bob_interfaces.srv import ImageRaster
import math 
import time 
import rclpy
from rclpy.node import Node
from bob_interfaces.srv import ImageRaster
import os
import json
import datetime
import re
##
class RasterPTZClient(Node):
    def __init__(self):
        os.chdir("/workspaces/bobcamera/src/ros2/Dropbox/Calibration")
        # Create a client for the ImageRaster service
        while(True):
            super().__init__('raster_ptz_client')
            json_files = [pos_json for pos_json in os.listdir(os.getcwd()) if (pos_json.endswith('.json') and not(pos_json.endswith('_done.json')))]
            if(len(json_files) > 0):
                self.client = self.create_client(ImageRaster, '/image_acquisition')
                with open(json_files[0]) as f:
                    RasterConfig = json.load(f)
                    print(RasterConfig)

                # Wait for the service to be available
                while not self.client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('ImageRaster service not available, waiting...')
                
                # Create a request object
                self.request = ImageRaster.Request()

                #For testing Purposes
                Rasterstep = 0
                completeMSG = False 
                initiate_v1 = True

                startX = RasterConfig['startX']
                endX = RasterConfig['endX']
                stepwidthX = RasterConfig['stepwidthX']
                startY = RasterConfig['startY']
                endY = RasterConfig['endY']
                stepwidthY = RasterConfig['stepwidthY']
                startZoom = RasterConfig['startZoom']
                endZoom = RasterConfig['endZoom']
                stepwidthZoom = RasterConfig['stepwidthZoom']
                campaign = RasterConfig['campaign']


                CampaignUnderlined = re.sub(r'[^a-zA-Z0-9_]', '_', campaign)
                TimeStartNotString = datetime.datetime.now()
                TimeStartFile= TimeStartNotString.strftime("%Y_%b_%d_%H_%M_%S")
                TimeStart= TimeStartNotString.strftime("%I:%M%p on %B %d, %Y")
                FileNameStart = f"{CampaignUnderlined}_{TimeStartFile}_in_progress.txt"
                RasteringProtocol = open(FileNameStart, "a")
                print("Rastering Specification:", file=RasteringProtocol)
                print(f"Campaign: {campaign}", file=RasteringProtocol)
                print(f"\tStart X: {startX}", file=RasteringProtocol)
                print(f"\tStart Y: {startY}", file=RasteringProtocol)
                print(f"\tStart Zoom: {startZoom}", file=RasteringProtocol)
                print(f"\tEnd X: {endX}", file=RasteringProtocol)
                print(f"\tEnd Y: {endY}", file=RasteringProtocol)
                print(f"\tEnd Zoom: {endZoom}", file=RasteringProtocol)
                print(f"\tstepwidthX: {stepwidthX}", file=RasteringProtocol)
                print(f"\tstepwidthY: {stepwidthY}", file=RasteringProtocol)
                print(f"\tstepwidthZoom: {stepwidthZoom}", file=RasteringProtocol)
                print(f"\tRastering Starts: {TimeStart}", file=RasteringProtocol)

                print(f"Protocol:", file=RasteringProtocol)
                #actual operation goes here:
                print("###################################", file=RasteringProtocol)
                print("Operation Protocol:", file=RasteringProtocol)
                        
                while(completeMSG == False):
                    if( (initiate_v1 == True)  or  (RasterImageACK_v1 == True)):
                        #initially magic numbers, later ros-msg-values:
                        XIncrementsPerY = math.ceil(abs(startX-endX)/stepwidthX)+1
                        YStepsTotal = math.ceil(abs(startY-endY)/stepwidthY)+1
                        if(Rasterstep < XIncrementsPerY*YStepsTotal):
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
                            


                            moverequest.Position.PanTilt.x = min(max(currentStepX,XMIN),XMAX)
                            moverequest.Position.PanTilt.y = min(max(currentStepY,YMIN),YMAX)
                            moverequest.Position.Zoom.x = min(max(startZoom,ZoomMIN),ZoomMAX)

                            #global active
                            if active:
                                ptz.Stop({'ProfileToken': moverequest.ProfileToken})
                            active = True  
                            ptz.AbsoluteMove(moverequest)
                



                            ####################################################################################
                            
                            
                            #Polling ONVIF if target has been reached
                            #while((ONVIFGetprofile.X != currentStepX) and (ONVIFGetprofile.Y != currentStepY)):
                                #time.sleep(0.1)
                            if(Rasterstep == 0):
                                time.sleep(5.0)
                            else:
                                time.sleep(3.0)

        
                            # Set the values of the request
                            self.request.x = moverequest.Position.PanTilt.x   # Replace with the actual values
                            self.request.y = moverequest.Position.PanTilt.y
                            self.request.zoom =  moverequest.Position.Zoom.x
                            self.request.campaign = CampaignUnderlined

                            print(f"Ros2 MSG calibrate/v1, CurrentStepX: {currentStepX}, CurrentStepY: {currentStepY}",  file=RasteringProtocol)
                            # Call the service
                            self.call_service()

                        else:
                            completeMSG = True
                            #####################
                            # ToDo: !!!!!!!!  ###
                            print("ROS2 MSG bob/ptz/calibrate/v1/complete.msg ACK")
                            #####################
                            print("###################################", file=RasteringProtocol)
                            #prints of current status go into file=f)
                            time.sleep(1)
                            #end operation goes here:
                            TimeNowNotString = datetime.datetime.now()
                            TimeNow = TimeNowNotString.strftime("%I:%M%p on %B %d, %Y")
                            print(f"Rastering completed: {TimeNow}", file=RasteringProtocol)
                            Duration = (TimeNowNotString-TimeStartNotString)
                            print(f"Rastering duration: {str(Duration)}", file=RasteringProtocol)
                            RasteringProtocol.close()
                            os.rename(json_files[0], f"{json_files[0][:-5]}_done.json")
                            os.rename(RasteringProtocol.name, f"{CampaignUnderlined}_{TimeStartFile}_complete.txt")

                    #For testing purposes:
                    initiate_v1 = False
                    RasterImageACK_v1 = False
                    time.sleep(0.2)
                    RasterImageACK_v1 = True
                    if completeMSG == False:
                        print(f"MSG RasterImageACK_v1 = {RasterImageACK_v1}")
                    Rasterstep += 1
            else:
                time.sleep(10.0)

    def call_service(self):
        # Call the service with the request
        future = self.client.call_async(self.request)

        # Wait for the service to complete
        rclpy.spin_until_future_complete(self, future)

        # Process the response
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Service call result: {response.success}")
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    node = RasterPTZClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()