import cv2
import numpy as np
import math
import rclpy
import rclpy.logging

# Day and night algorithms based on:
# Li, Qingyong & Lyu, Weitao & Yang, Jun. (2011). A Hybrid Thresholding Algorithm for Cloud Detection on Ground-Based Color Images. Journal of Atmospheric and Oceanic Technology. 28. 
# Gao, B.; Ping, Y.; Lu, Y.; Zhang, C. (2022) Nighttime Cloud Cover Estimation Method at the Saishiteng 3850 m Site. Universe, 8, 538. 
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
   
    def find_threshold_mce(self, img, mask):
        StArray = img[mask]

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
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_frame, 1, 255, cv2.THRESH_BINARY)     
        return mask
    
class DayTimeCloudEstimator(CloudEstimator):

    def __init__(self):
        super().__init__()
        pass
    
    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):
        # logger = rclpy.logging.get_logger('cloud_estimator')
        mask = self.get_mask(frame)
        
        b, g, r = cv2.split(frame)
        r = np.where(r == 0, 1, r)
        r = r.astype(np.float32)
        b = b.astype(np.float32)
        lambda_n = np.where(mask, (b - r) / (b + r), 0)

        std = np.std(lambda_n[mask == 255])
        # logger.info(f'std: {std}')
        if std > 0.03: # magic number, could maybe replace with Dip Test of modality
            # logger.info('Bimodal distribution')
            threshold = self.find_threshold_mce(lambda_n, mask)
            _, ratio_mask = cv2.threshold(lambda_n, threshold, 255, cv2.THRESH_BINARY)
            distribution_type = False 
        else:
            # logger.info('Unimodal distribution')
            _, ratio_mask = cv2.threshold(b-r, 30, 255, cv2.THRESH_BINARY)
            distribution_type = True 
            
        # Count the number of pixels in each part of the mask
        N_Cloud = np.count_nonzero(ratio_mask[mask == 255] == 0)
        N_Sky = np.count_nonzero(ratio_mask[mask == 255] == 255)  
        
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100        
        return round(ccr, 2), distribution_type

class NightTimeCloudEstimator(CloudEstimator):

    def __init__(self):
        super().__init__()
        pass

    def generate_cloud_feature_image(self, MI):
        R = 1.164 * (MI - 16) + 1.596 * (MI - 128)
        G = 1.164 * (MI - 16) - 0.392 * (MI - 128) - 0.813 * (MI - 128)
        B = 1.164 * (MI - 16) + 2.017 * (MI - 128)
        
        max_val = np.maximum(R, np.maximum(G, B))
        min_val = np.minimum(R, np.minimum(G, B))
       
        cloud_feature_image = (max_val - min_val) / max_val
        cloud_feature_image_uint8 = (cloud_feature_image * 255).astype(np.uint8)

        return cloud_feature_image_uint8

    # Method to estimate the cloudyness of the frame
    def estimate(self, frame):

        mask = self.get_mask(frame)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cloud_feature_image = self.generate_cloud_feature_image(frame_gray[mask == 255])
        otsu_threshold, image_result = cv2.threshold(cloud_feature_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # Count the number of non zero pixels (white)
        N_Cloud = np.count_nonzero(image_result)
        N_Sky = np.count_nonzero(image_result == 0)
    
        ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100
        return round(ccr, 2), True  
