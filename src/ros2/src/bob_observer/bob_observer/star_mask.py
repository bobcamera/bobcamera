import cv2
import numpy as np
import rclpy
import rclpy.logging

class StarMaskMaker():

    @staticmethod
    def MaskMaker(threshold):
        return StarMaskGenerator(threshold)

    def __init__(self):
        self.logger = rclpy.logging.get_logger('star_mask_generator')# .info(f'Running node {self.node.get_name()} via the node runner')
    
    def process_image(self, frame):
        pass
    
    
class StarMaskGenerator:
    def __init__(self, threshold):
        if threshold < 0.3 or threshold > 0.95:
            raise ValueError("Star mask threshold must be in the range of 0.3 to 0.95")
        self.circle_radius = 5  
        self.blurred_circle = self._create_blurred_circle(self.circle_radius)
        self.threshold = threshold

    def _create_blurred_circle(self, radius):
        
        x, y = np.meshgrid(np.arange(self.circle_radius*2), np.arange(self.circle_radius*2))
        circle = np.sqrt((x - self.circle_radius)**2 + (y - self.circle_radius)**2) <= self.circle_radius
        circle = circle.astype(np.float64) 
        circle /= np.max(circle) 
        background = np.zeros((50, 50))
        circle_position = ((background.shape[0] - circle.shape[0]) // 2, (background.shape[1] - circle.shape[1]) // 2)
        background[circle_position[0]:circle_position[0]+self.circle_radius*2, circle_position[1]:circle_position[1]+self.circle_radius*2] = circle

        sigma = 1 
        kernel_size = int(np.ceil(6*sigma))  
        if kernel_size % 2 == 0:
            kernel_size += 1  
        gaussian_kernel = cv2.getGaussianKernel(kernel_size, sigma)
        gaussian_kernel = np.outer(gaussian_kernel, gaussian_kernel)
        blurred_circle = cv2.filter2D(background, -1, gaussian_kernel, borderType=cv2.BORDER_REPLICATE)

        return blurred_circle

    def normxcorr2(self, template, image):
        if np.ndim(template) > np.ndim(image) or \
                len([i for i in range(np.ndim(template)) if template.shape[i] > image.shape[i]]) > 0:
            print("normxcorr2: TEMPLATE larger than IMG. Arguments may be swapped.")
            
        template = template - np.mean(template) # Could compute mean in init 
        image = image - np.mean(image) 

        a1 = np.ones(template.shape)
        ar = np.flipud(np.fliplr(template))
        out = cv2.filter2D(image, cv2.CV_64F, ar.conj(), borderType=cv2.BORDER_REPLICATE)

        image_squared_conv = cv2.filter2D(np.square(image), cv2.CV_64F, a1, borderType=cv2.BORDER_REPLICATE)
        image_squared_conv -= np.square(out) / np.prod(template.shape)
        image_squared_conv[image_squared_conv < 0] = 0

        template_squared_sum = np.sum(np.square(template))
        with np.errstate(divide='ignore', invalid='ignore'): 
            out /= np.sqrt(image_squared_conv * template_squared_sum)

        # Remove any divisions by 0 or very close to 0
        out[~np.isfinite(out)] = 0
        
        return out

    def process_image(self, frame):
        if len(frame.shape) == 2:
            frame_gray = frame
        else:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blurred_circle_gray = self.blurred_circle.astype(np.float32)
        frame_gray = frame_gray.astype(np.float32)

        norm_correlation = self.normxcorr2(blurred_circle_gray, frame_gray)

        binary_mask = norm_correlation >= self.threshold
        binary_mask_uint8 = binary_mask.astype(np.uint8) * 255

        # Dilate
        kernel = np.ones((16, 16), np.uint8)
        dilated_binary_mask = cv2.dilate(binary_mask_uint8, kernel, iterations=1)
        resized_dilated_binary_mask = cv2.resize(dilated_binary_mask, (frame.shape[1], frame.shape[0]))

        inverted_binary_mask = cv2.bitwise_not(resized_dilated_binary_mask)
        return inverted_binary_mask


# # Test code 
# if __name__ == "__main__":
#     roi = cv2.imread("stars_ptz.png")
#     star_mask = StarMaskGenerator(0.5)
#     inverted_binary_mask = star_mask.process_image(roi)
    
#     inverted_binary_mask_resized = cv2.resize(inverted_binary_mask, None, fx=0.25, fy=0.25)
#     cv2.imshow('inverted_binary_mask', inverted_binary_mask_resized)
#     cv2.waitKey(0)
    
#     roi_resized = cv2.resize(roi, None, fx=0.25, fy=0.25)
#     cv2.imshow('Original', roi_resized)
#     cv2.waitKey(0)
    
#     # Draw circles around stars on the original image
#     for coord in np.argwhere(inverted_binary_mask == 0):
#         cv2.circle(roi, (coord[1], coord[0]), 5, (0, 255, 0), 1)

#     roi_resized = cv2.resize(roi, None, fx=0.25, fy=0.25)
#     cv2.imshow('Original with identified stars', roi_resized)
#     cv2.waitKey(0)
    
    # cv2.destroyAllWindows()