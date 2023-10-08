import cv2
import numpy as np
import math
import rclpy
import rclpy.logging

class CloudEstimator():

    @staticmethod
    def Day():
        return DayTimeCloudEstimator()

    @staticmethod
    def Night():
        return NightTimeCloudEstimator()

    def __init__(self):
        self.logger = rclpy.logging.get_logger('cloud_estimator')# .info(f'Running node {self.node.get_name()} via the node runner')
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):
        pass

    def br_norm(self, img, mask):
        b, g, r = cv2.split(img)
        r = np.where(r == 0, 1, r)
        r = r.astype(np.float32)
        b = b.astype(np.float32)
        lambda_n = np.where(mask, (b - r) / (b + r), 0)
        return lambda_n
    
    def find_threshold_mce(self, img):

        StArray = img.ravel()
        x = np.linspace(-1,1,201)
        y, bins = np.histogram(StArray, bins=x)

        MinValue = np.min(StArray)
        MaxValue = np.max(StArray)

        t_int_decimal= MinValue+((MaxValue-MinValue)/2)
        t_int = np.ceil(t_int_decimal*100)/100
        index_of_t_int = np.argmin(np.abs(x - t_int))

        m0a = 0
        m1a = 0
        for i in range(index_of_t_int):
            m0a += y[i]
            m1a += x[i] * y[i]
        
        m0b = 0
        m1b = 0
        for i in range(index_of_t_int, 200):
            m0b += y[i]
            m1b += x[i] * y[i]

        mu_a=(m1a/m0a)
        mu_b=(m1b/m0b)

        if mu_a < 0:
            mu_a = abs(mu_a)

        diff=5
        t_n=0

        t_n_decimal=((mu_b-mu_a) /(np.log(mu_b)-np.log(mu_a)))
        if math.isnan(t_n_decimal) == False:
            t_n = np.ceil(t_n_decimal*100)/100; 

        iter = 1
        while True:
            t_int = t_n
        
            # Finding index of t_int
            for i in range(201):
                if x[i] == t_int:
                    index_of_t_int = i
                    break
        
            m0a = 0
            m1a = 0
            for i in range(index_of_t_int):
                m0a += y[i]
                m1a += x[i] * y[i]
            
            m0b = 0
            m1b = 0
            for i in range(index_of_t_int, 200):
                m0b += y[i]
                m1b += x[i] * y[i]
        
            mu_a = m1a/m0a
            mu_b = m1b/m0b

            if mu_a < 0:
                mu_a = abs(mu_a)
        
            t_nplus1_decimal = (mu_b-mu_a)/(np.log(mu_b)-np.log(mu_a))
            if math.isnan(t_nplus1_decimal) == False:
                t_nplus1 = math.ceil(t_nplus1_decimal * 100) / 100
        
                diff = abs(t_nplus1 - t_n)
                t_n = t_nplus1
        
                if diff == 0:
                    break
        
                iter += 1

        ThresholdValue = t_n

        return ThresholdValue

    def get_mask(self, frame):
        height, width, _ = frame.shape
        x, y = np.ogrid[:height, :width]
        center_x, center_y = width // 2, height // 2
        radius = int(width * 0.315)
        mask = (x - center_x)**2 + (y - center_y)**2 < radius**2
        return mask        

class DayTimeCloudEstimator(CloudEstimator):

    def __init__(self):
        super().__init__()
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):

        mask = self.get_mask(frame)

        lambda_n = self.br_norm(frame, mask)

        std = np.std(lambda_n[mask])
        if std > 0.03: # magic number
            #print("Bimodal distribution")
            threshold = self.find_threshold_mce(lambda_n)
            _, ratio_mask = cv2.threshold(lambda_n, threshold, 255, cv2.THRESH_BINARY)
        else:
            #print("Unimodal distribution")
            _, ratio_mask = cv2.threshold(lambda_n, 0.25, 255, cv2.THRESH_BINARY)

        # Count the number of pixels in each part of the mask
        N_Cloud = np.count_nonzero(ratio_mask[mask] == 0)
        N_Sky = np.count_nonzero(ratio_mask[mask])

        # Cloud cover ratio
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100
        return round(ccr,2)

class NightTimeCloudEstimator(CloudEstimator):

    def __init__(self):
        super().__init__()
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):

        mask = self.get_mask(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        otsu_threshold, image_result = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU,)

        # Count the number of non zero pixels (white)
        N_Cloud = np.count_nonzero(image_result[mask])
        N_Sky = np.count_nonzero(image_result[mask] == 0)
    
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100
        return round(ccr,2)
