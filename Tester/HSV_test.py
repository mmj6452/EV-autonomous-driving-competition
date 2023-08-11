import cv2
import numpy as np


lower_white = np.array([0,0,240],dtype=np.uint8)
upper_white = np.array([255,255,255],dtype=np.uint8)
upper_yellow = np.array([30,0,0],dtype=np.uint8)
lower_yellow = np.array([90,255,255],dtype=np.uint8)


img_bgr = cv2.imread("/home/ubuntu/2705763659488FB401.png")
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
cv2.imshow("BGR", img_bgr)
img_white_line  = cv2.inRange(img_hsv, lower_white, upper_white)
cv2.imshow("White line", img_white_line)
img_yellow_line = cv2.inRange(img_hsv, upper_yellow, lower_yellow)
cv2.imshow("Yellow line", img_yellow_line)
cv2.waitKey(0)
lower_yellow