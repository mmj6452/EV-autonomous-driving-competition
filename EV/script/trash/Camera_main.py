#Lane Traking으로 만들던 파일 현재는 Lane_drive.py로 대체


#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import cv2
import os

#HSV색 범위 지정
lower_white = np.array([0,0,200],dtype=np.uint8)
upper_white = np.array([255,255,255],dtype=np.uint8)
upper_yellow = np.array([30,0,0],dtype=np.uint8)
lower_yellow = np.array([90,255,255],dtype=np.uint8)

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class Get_Camera:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

        self.source_prop = np.float32([[0.01, 0.80],
                                       [0.5 - 0.14, 0.30],
                                       [0.5 + 0.14, 0.30],
                                       [1 - 0.01, 0.80]
                                       ])
    def warp_image(self,img, source_prop):
        
            image_size = (img.shape[1], img.shape[0])
            x = img.shape[1]
            y = img.shape[0]
            destination_points = np.float32([
            [0, y],
            [0, 0],
            [x, 0],
            [x, y]
            ])
            source_points = source_prop * np.float32([[x, y]]* 4)
            perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)
            warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
            return warped_img
    def get_line_img(self, img):
        img_hsv = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
        #흰색과 노란색을 추출
        img_white_line  = cv2.inRange(img_hsv, lower_white, upper_white)
        img_yellow_line = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        img_line = cv2.bitwise_or(img_white_line, img_yellow_line)
        return img_line
    def draw_line(self,img):

        canny = cv2.Canny(img, 5000, 1500, apertureSize = 5, L2gradient = True)
        lines = cv2.HoughLinesP(canny, 0.8, np.pi / 180, 100, minLineLength = 50, maxLineGap = 100)
        img_line = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        if lines is not None:
            for i in lines:
                cv2.line(img_line, (int(i[0][0]), int(i[0][1])), (int(i[0][2]), int(i[0][3])), (0, 0, 255), 2)
        cv2.imshow("Line Image", img_line)
        return img_line
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.height , self.width = self.img_bgr.shape[:2]
        except CvBridgeError as e:
            print(e)
        roi_img = self.img_bgr[150:480, 0:640]
        #img_warp = self.warp_image(self.img_bgr, self.source_prop)
        img_line = self.get_line_img(self.img_bgr)
        # line_detect
        img_line = self.draw_line(img_line)
        dst = cv2.addWeighted(self.img_bgr, 1, img_line, 1, 0)
        cv2.imshow("Image window", dst)
        cv2.waitKey(1)
        
if __name__ == '__main__':
    rospy.init_node('Get_img', anonymous=True)
    camera = Get_Camera()
    rospy.spin()
