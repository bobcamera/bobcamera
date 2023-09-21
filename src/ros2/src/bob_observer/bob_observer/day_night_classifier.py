import cv2
import numpy as np
import rclpy
import rclpy.logging
from bob_shared.enumerations import DayNightEnum

class DayNightEstimator():

    @staticmethod
    def Classifier(threshold):
        return DayNightClassifier(threshold)

    def __init__(self):
        self.logger = rclpy.logging.get_logger('day_night_classifier')# .info(f'Running node {self.node.get_name()} via the node runner')
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):
        pass

# This classifier has been taken from https://github.com/arunnthevapalan/day-night-classifier/blob/master/classifier.ipynb
# This classifier is to try and determine if an image is classified as day or night
class DayNightClassifier(DayNightEstimator):

    def __init__(self, threshold):
        super().__init__()

        self.threshold = threshold

    # This function should take in RGB image input
    def estimate(self, frame):

        result: DayNightEnum = DayNightEnum.Night

        h, w, _ = frame.shape
        area = h*w  # pixels
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Add up all the pixel values in the V channel
        sum_brightness = np.sum(hsv_frame[:,:,2])

        # Extract average brightness feature from an HSV image
        # Find the average Value or brightness of an image
        avg_brightness = int(sum_brightness/area)

        if(avg_brightness > self.threshold):
        # if the average brightness is above the threshold value, we classify it as "day"
            result = DayNightEnum.Day

        #self.logger.info(f'Average brightness: {avg_brightness}, threshhold: {self.threshold}, classifier: {result}')

        return result, avg_brightness
