import cv2

def scale_image(frame, w, h):
    if frame.shape[0] > h or frame.shape[1] > w:
        # calculate the width and height percent of original size
        width = int((w / frame.shape[1]) * 100)
        height = int((h / frame.shape[0]) * 100)
        # pick the largest of the two
        scale_percent = max(width, height)
        # calc the scaled width and height
        scaled_width = int(frame.shape[1] * scale_percent / 100)
        scaled_height = int(frame.shape[0] * scale_percent / 100)
        return cv2.resize(frame, (scaled_width, scaled_height))
    else:
        return frame