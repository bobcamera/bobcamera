import os
import json
import datetime
import time
import re

         

os.chdir("/workspaces/bobcamera/src/ros2/Dropbox/Calibration")


json_files = [pos_json for pos_json in os.listdir(os.getcwd()) if (pos_json.endswith('.json') and not(pos_json.endswith('_done.json')))]


XMAX = 1
XMIN = -1
YMAX = 1
YMIN = -1
ZoomMAX = 10
ZoomMIN = 0


with open(json_files[0]) as f:
    RasterConfig = json.load(f)
    print(RasterConfig)

startX = RasterConfig['startX']
endX = RasterConfig['endX']
stepwidthX = RasterConfig['stepwidthX']
startY = RasterConfig['startY']
endY = RasterConfig['endY']
stepwidthY = RasterConfig['stepwidthY']
campaign = RasterConfig['campaign']
zoom = RasterConfig['zoom']

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
print(f"\tEnd X: {endX}", file=RasteringProtocol)
print(f"\tEnd Y: {endY}", file=RasteringProtocol)
print(f"\tstepwidthX: {stepwidthX}", file=RasteringProtocol)
print(f"\tstepwidthY: {stepwidthY}", file=RasteringProtocol)
print(f"\tzoom: {zoom}", file=RasteringProtocol)
print(f"\tRastering Starts: {TimeStart}", file=RasteringProtocol)

print(f"Protocol:", file=RasteringProtocol)
#actual operation goes here:
print("###################################", file=RasteringProtocol)
print("Actual text output of each command goes here", file=RasteringProtocol)
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




