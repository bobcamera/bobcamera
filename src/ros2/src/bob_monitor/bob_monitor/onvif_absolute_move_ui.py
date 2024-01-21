import asyncio, sys
from onvif2 import ONVIFCamera


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
def do_move(ptz, request):
    # Start continuous move
    global active
    if active:
        ptz.Stop({'ProfileToken': request.ProfileToken})
    active = True  
    #print(request)
    #print(type(request))
    #ptz.RelativeMove({'ProfileToken': 'Profile_1','Translation': {'PanTilt': {'x': 0.2,'y': 0},'Zoom': {'x': 0.0}}})
    ptz.AbsoluteMove(request)

def move_up(ptz, request):
    print ('move to door...')
    request.Position.PanTilt.x = 0
    request.Position.PanTilt.y = YMAX
    do_move(ptz, request)

def move_down(ptz, request):
    print ('move to sky...')
    request.Position.PanTilt.x = 0
    request.Position.PanTilt.y = YMIN
    do_move(ptz, request)

def move_right(ptz, request):
    print ('move right...')
    request.Position.PanTilt.x = XMAX
    request.Position.PanTilt.y = 0
    do_move(ptz, request)

def move_left(ptz, request):
    print ('move to tree...')
    request.Position.PanTilt.x = XMIN
    request.Position.PanTilt.y = 0
    do_move(ptz, request)
    

def move_upleft(ptz, request):
    print ('move to door...')
    request.Position.PanTilt.x = XMIN
    request.Position.PanTilt.y = YMAX
    do_move(ptz, request)
    
def move_upright(ptz, request):
    print ('move to fisheye...')
    request.Position.PanTilt.x = XMAX
    request.Position.PanTilt.y = YMAX
    do_move(ptz, request)
    
def move_downleft(ptz, request):
    print ('move down left...')
    request.Position.PanTilt.x = XMIN
    request.Position.PanTilt.y = YMIN
    do_move(ptz, request)
    
def move_downright(ptz, request):
    print ('move down left...')
    request.Position.PanTilt.x = XMAX
    request.Position.PanTilt.y = YMIN
    do_move(ptz, request)

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

def readin():
    """Reading from stdin and displaying menu"""
    global moverequest, ptz
    
    selection = sys.stdin.readline().strip("\n")
    lov=[ x for x in selection.split(" ") if x != ""]
    if lov:
        
        if lov[0].lower() in ["u","up", "door"]:
            move_up(ptz,moverequest)
        elif lov[0].lower() in ["d","do","dow","down", "sky"]:
            move_down(ptz,moverequest)
        elif lov[0].lower() in ["l","le","lef","left", "links", "tree"]:
            move_left(ptz,moverequest)
        elif lov[0].lower() in ["r","ri","rig","righ","right", "rihgt", "rechts"]:
            move_right(ptz,moverequest)
        elif lov[0].lower() in ["ul", "fisheye"]:
            move_upleft(ptz,moverequest)
        elif lov[0].lower() in ["ur"]:
            move_upright(ptz,moverequest)
        elif lov[0].lower() in ["dl"]:
            move_downleft(ptz,moverequest)
        elif lov[0].lower() in ["dr"]:
            move_downright(ptz,moverequest)
        elif lov[0].lower() in ["s","st","sto","stop"]:
            ptz.Stop({'ProfileToken': moverequest.ProfileToken})
            active = False
        else:
            print("What are you asking?\tI only know, 'sky', 'door', 'fisheye', 'tree' and 'stop'")
         
    print("")
    print("Your command: ", end='',flush=True)
       
            
if __name__ == '__main__':
    setup_move()
    loop = asyncio.get_event_loop()
    try:
        loop.add_reader(sys.stdin,readin)
        print("Use Ctrl-C to quit")
        print("Your command: ", end='',flush=True)
        loop.run_forever()
    except:
        pass
    finally:
        loop.remove_reader(sys.stdin)
        loop.close()
