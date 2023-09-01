import cv2

###########################################################################################################
# This code file contains a selection of useful utility functions that are used throughout simple tracker #
###########################################################################################################

# Utility function to determine if the installed version of open cv is supported
# We support v4.1 and above
def is_cv_version_supported():
    (major_ver, minor_ver, subminor_ver) = get_cv_version()
    if int(major_ver) >= 4 and int(minor_ver) >= 1:
        return True
    return False

# Utility function to get the installed version of open cv
def get_cv_version():
    return (cv2.__version__).split('.')

# Utility function to determine if cuda is supported
def is_cuda_supported(): # 1 == using cuda, 0 = not using cuda
    enabled = False
    try:
        count = cv2.cuda.getCudaEnabledDeviceCount()
        if count > 0:
            enabled = True
    except:
        pass
    return enabled

# Utility function to get the video writer in a standardised way
def get_writer(output_filename, width, height):
    print(f'source w,h:{(width, height)}')
    return cv2.VideoWriter(output_filename, cv2.VideoWriter_fourcc(*"AVC1"), 30, (width, height))

# Utility function to convert jey points in to a bounding box
# The bounding box is used for track validation (if enabled) and will be displayed by the visualiser
# as it tracks a point of interest (blob) on the frame
def kp_to_bbox(kp):
    (x, y) = kp.pt
    size = kp.size
    scale = 6
    #print(f'kp_to_bbox x, y:{(x, y)}, size:{size}, scale:{scale}, new size:{scale * size}')
    return (int(x - scale * size / 2), int(y - scale * size / 2), int(scale * size), int(scale * size))

# Utility function to determine if 2 bounding boxes overlap each other. In order to make tracking more efficient
# we try not to track sections of the same point of interest (blob)
def bbox_overlap(bbox1, bbox2):
    #    bb1 : dict
    #        Keys: {'x1', 'x2', 'y1', 'y2'}
    #        The (x1, y1) position is at the top left corner,
    #        the (x2, y2) position is at the bottom right corner
    #    bb2 : dict
    #        Keys: {'x1', 'x2', 'y1', 'y2'}
    #        The (x, y) position is at the top left corner,
    #        the (x2, y2) position is at the bottom right corner

    bb1 = {}
    bb1['x1'] = bbox1[0]
    bb1['y1'] = bbox1[1]
    bb1['x2'] = bbox1[0] + bbox1[2]
    bb1['y2'] = bbox1[1] + bbox1[3]

    bb2 = {}
    bb2['x1'] = bbox2[0]
    bb2['y1'] = bbox2[1]
    bb2['x2'] = bbox2[0] + bbox2[2]
    bb2['y2'] = bbox2[1] + bbox2[3]

    assert bb1['x1'] <= bb1['x2']
    assert bb1['y1'] <= bb1['y2']
    assert bb2['x1'] <= bb2['x2']
    assert bb2['y1'] <= bb2['y2']

    # determine the coordinates of the intersection rectangle
    x_left = max(bb1['x1'], bb2['x1'])
    y_top = max(bb1['y1'], bb2['y1'])
    x_right = min(bb1['x2'], bb2['x2'])
    y_bottom = min(bb1['y2'], bb2['y2'])

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    # The intersection of two axis-aligned bounding boxes is always an
    # axis-aligned bounding box.
    # NOTE: We MUST ALWAYS add +1 to calculate area when working in
    # screen coordinates, since 0,0 is the top left pixel, and w-1,h-1
    # is the bottom right pixel. If we DON'T add +1, the result is wrong.
    intersection_area = (x_right - x_left + 1) * (y_bottom - y_top + 1)

    # compute the area of both AABBs
    bb1_area = (bb1['x2'] - bb1['x1'] + 1) * (bb1['y2'] - bb1['y1'] + 1)
    bb2_area = (bb2['x2'] - bb2['x1'] + 1) * (bb2['y2'] - bb2['y1'] + 1)

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
    assert iou >= 0.0
    assert iou <= 1.0
    return iou

# Utility function to determine if a bounding box 1 contains bounding box 2. In order to make tracking more efficient
# we try not to track sections of the same point of interest (blob)
def bbox1_contain_bbox2(bbox1, bbox2):
    x1, y1, w1, h1 = bbox1
    x2, y2, w2, h2 = bbox2
    return (x2 > x1) and (y2 > y1) and (x2+w2 < x1+w1) and (y2+h2 < y1+h1)

# Utility function to calculate the distance between the centre points of 2 bounding boxes
def calc_centre_point_distance(bbox1, bbox2):
    x1, y1, w1, h1 = bbox1
    c1 = (int(x1+(w1/2)), int(y1+(h1/2)))
    x2, y2, w2, h2 = bbox2
    c2 = (int(x2+(w2/2)), int(y2+(h2/2)))
    #euclidean = math.sqrt((x2-x1)**2+(y2-y1)**2)
    res = cv2.norm(c1, c2)
    return int(res)

# Utility function to determine if a bounding box is already being tracked by checkling if its overlapped or already contained
def is_bbox_being_tracked(live_trackers, bbox):
    # Mike: The bbox contained should computationally be faster than the overlap, so we use it first as a shortcut
    for tracker in live_trackers:
        if tracker.is_bbox_contained(bbox):
            return True
        else:
            if tracker.does_bbox_overlap(bbox):
                return True
    return False

# Utility function to deletrmine if a point overlaps a bouding box
def is_point_contained_in_bbox(bbox, point):
    x, y, w, h = bbox
    x0, y0 = point
    return x <= x0 < x + w and y <= y0 < y + h

# Utility function to get the sized bbox from tracker for display
def get_sized_bbox_from_tracker(tracker):
    return get_sized_bbox(tracker.get_bbox(), tracker.settings)

# Utility function to get the sized bbox for display
# bbox_fixed_size is not defined in App settings yet
# this function is not being used anywhere (similar method exists in the Annotated Frame Creator
# I'm commenting the old function out for now and replacing it with a new one. The new one is still not used yet. -DL
# def get_sized_bbox(bbox, settings):
#     return_bbox = bbox
#     if settings['bbox_fixed_size']:
#         size = settings['visualiser_bbox_size']
#         x1, y1, w, h = bbox
#         x1 = int(x1+(w/2)) - int(size/2)
#         y1 = int(y1+(h/2)) - int(size/2)
#         return_bbox = (x1, y1, size, size)
#     return return_bbox

def get_sized_bbox(bbox, settings):
    x1, y1, w, h = bbox
    size_setting = settings.get('visualiser_bbox_size')  # This won't raise a KeyError if the key doesn't exist
    if isinstance(size_setting, (int, float)):  # Check if the value is a number
        size = max(w, h, size_setting)
    else:
        size = max(w, h)
    x1 = int(x1 + (w / 2)) - int(size / 2)
    y1 = int(y1 + (h / 2)) - int(size / 2)
    return_bbox = (x1, y1, size, size)
    return return_bbox

# Utility function to determine if a bbox is valid.
#
# I had problems and used this to debug it, its a useful function to have so don't want to delete
# it, but equally I don't want the processing overhead so quick return out of it until we need it again.
def is_valid_bbox(bbox, frame):
    #return True
    # roi.x + roi.width <= m.cols
    # roi.y + roi.height <= m.rows
    valid = False
    x, y, w, h = bbox
    f_h, f_w, _ = frame.shape
    max_dim = int(f_w*0.15) # Make this 15% of the width of the frame

    if w < max_dim > h:
        if (x > 1 and y > 1 and w > 1 and h > 1):
            f_h, f_w, _ = frame.shape
            cols = x + w
            rows = y + h
            if (cols < f_w and rows < f_h):
                valid = True
    return valid

# Utility function to take a frame and return a smaller one
# (size divided by zoom level) centered on center
def zoom_and_clip(frame, center, zoom_level):
    height, width, _channels = frame.shape

    new_height = int(height/zoom_level)
    new_width = int(width/zoom_level)

    return clip_at_center(frame, center, width, height, new_width, new_height)

# Utility function to clip out the center part of a frame. This is mainly used by the fish-eye mask
# to remove masked ("black") parts of the frame
def clip_at_center(frame, center, width, height, new_width, new_height):
    x, y = center
    half_width = int(new_width/2)
    half_height = int(new_height/2)

    left = max(0, x-half_width)
    right = min(x+half_width, width)
    right = max(new_width, right)

    top = max(0, y-half_height)
    bottom = min(y+half_height, height)
    bottom = max(new_height, bottom)

    return frame[top:bottom, left:right]

def frame_resize(frame, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the frame to be resized and
    # grab the frame size
    dim = None
    (h, w) = frame.shape[:2]

    # if both the width and height are None, then return the
    # original frame
    if width is None and height is None:
        return frame

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the frame
    resized = cv2.resize(frame, dim, interpolation = inter)

    # return the resized frame
    return resized

def get_optimal_font_scale(text, width):
    for scale in reversed(range(0, 60, 1)):
        textSize = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=scale/10, thickness=1)
        new_width = textSize[0][0]
        if (new_width <= width):
            return scale/10
    return 1
