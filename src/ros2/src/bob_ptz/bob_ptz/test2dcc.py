import cv2 as cv2

img = cv2.imread("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/fisheye_shot_2dcc.png")

# write image, example: 
# cv2.imwrite("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/images_output/fisheye_shot_2dc
# c.png", img)
#template= cv2.imread("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/ptz_shot_2dcc.png")

template= cv2.imread("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/ptz7.jpeg")

#methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED',     'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
#for i in range(len(methods)):
    #result[i] = cv2.matchTemplate(img,template,methods[i])
    #print ("Method {}  : Result{}") .format(method[i],result[i])

downscaling_factor = 6
down_width = round(template.shape[1]/downscaling_factor)
down_height = round(template.shape[0]/downscaling_factor)
down_points = (down_width, down_height)
template_downsized = cv2.resize(template, down_points, interpolation = cv2.INTER_LINEAR)
cv2.imwrite("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/images_output/template_downsized.png", template_downsized)
imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
templateGray = cv2.cvtColor(template_downsized, cv2.COLOR_BGR2GRAY)

result = cv2.matchTemplate(imgGray, templateGray, cv2.TM_CCORR_NORMED)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
#cv2.imwrite("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/images_output/result.png", result)

(startX, startY) = maxLoc 
endX = startX + template_downsized.shape[1]
endY = startY + template_downsized.shape[0]

cv2.rectangle(img, (startX, startY), (endX, endY), (255,0,0),3)
cv2.imwrite("/workspaces/bobcamera/src/ros2/src/bob_ptz/bob_ptz/images_ptz_and_fisheye/images_output/result.png", img)
cv2.waitKey(0)