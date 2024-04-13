import cv2
import numpy as np

# Based on Matlab port of normalised 2D cross correlation here:
# https://github.com/Sabrewarrior/normxcorr2-python/tree/master
# Optimised further here by using filter2D instead of scipy fftconvolve 
def normxcorr2(template, image):
    
    if np.ndim(template) > np.ndim(image) or \
            len([i for i in range(np.ndim(template)) if template.shape[i] > image.shape[i]]) > 0:
        print("normxcorr2: TEMPLATE larger than IMG. Arguments may be swapped.")
        
    template = template - np.mean(template) # Could pre-compute this once and pass in
    image = image - np.mean(image) 

    a1 = np.ones(template.shape)
    ar = np.flipud(np.fliplr(template))
    out = cv2.filter2D(image, cv2.CV_64F, ar.conj(), borderType=cv2.BORDER_REPLICATE)

    image_squared_conv = cv2.filter2D(np.square(image), cv2.CV_64F, a1, borderType=cv2.BORDER_REPLICATE)
    image_squared_conv -= np.square(out) / np.prod(template.shape)

    # Remove precision errors after subtraction
    image_squared_conv[image_squared_conv < 0] = 0

    template_squared_sum = np.sum(np.square(template))
    with np.errstate(divide='ignore', invalid='ignore'): 
        out /= np.sqrt(image_squared_conv * template_squared_sum)

    # Remove any divisions by 0 or very close to 0
    out[~np.isfinite(out)] = 0
    
    return out

def main():

    # region of interest (incoming masked detection frame)
    roi = cv2.imread("stars_fisheye.png")
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) 

    # Make a white dot and blur
    circle_radius = 5
    x, y = np.meshgrid(np.arange(circle_radius*2), np.arange(circle_radius*2))
    circle = np.sqrt((x - circle_radius)**2 + (y - circle_radius)**2) <= circle_radius
    circle = circle.astype(np.float64) 
    circle /= np.max(circle) 
    background = np.zeros((50, 50))
    circle_position = ((background.shape[0] - circle.shape[0]) // 2, (background.shape[1] - circle.shape[1]) // 2)
    background[circle_position[0]:circle_position[0]+circle_radius*2, circle_position[1]:circle_position[1]+circle_radius*2] = circle

    sigma = 1 
    kernel_size = int(np.ceil(6*sigma))  
    if kernel_size % 2 == 0:
        kernel_size += 1  
    gaussian_kernel = cv2.getGaussianKernel(kernel_size, sigma)
    gaussian_kernel = np.outer(gaussian_kernel, gaussian_kernel)
    blurred_circle = cv2.filter2D(background, -1, gaussian_kernel, borderType=cv2.BORDER_REPLICATE)

    # Ensure data type is correct
    blurred_circle_gray = blurred_circle.astype(np.float32)
    roi_gray = roi_gray.astype(np.float32)

    # Do 2D xcorr (fast version)
    norm_correlation = normxcorr2(blurred_circle_gray, roi_gray)

    # Threshold the result to obtain a binary mask
    threshold = 0.4
    binary_mask = norm_correlation >= threshold

    # Convert to uint8
    binary_mask_uint8 = binary_mask.astype(np.uint8) * 255

    # Dilate
    kernel = np.ones((16, 16), np.uint8)
    dilated_binary_mask = cv2.dilate(binary_mask_uint8, kernel, iterations=1)
    resized_dilated_binary_mask = cv2.resize(dilated_binary_mask, (roi.shape[1], roi.shape[0]))

    # Apply to mask
    inverted_binary_mask = cv2.bitwise_not(resized_dilated_binary_mask)

    # Read the original mask image
    mask = cv2.imread("mask.jpg", cv2.IMREAD_GRAYSCALE)

    # Combine the original mask with the inverted binary mask
    combined_mask = cv2.bitwise_and(mask, inverted_binary_mask)
    
    combined_mask_resized = cv2.resize(combined_mask, None, fx=0.25, fy=0.25) 
    cv2.imshow('New mask', combined_mask_resized)
    cv2.waitKey(0)

    # Draw circles around stars on the original image
    for coord in np.argwhere(binary_mask_uint8 == 255):
        cv2.circle(roi, (coord[1], coord[0]), 12, (0, 255, 0), 1)

    roi_resized = cv2.resize(roi, None, fx=0.25, fy=0.25)
    cv2.imshow('Original and Resized Binary Mask', roi_resized)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
