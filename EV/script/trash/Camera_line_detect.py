#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 카메라에서 차선을 인식하는 코드
# 노란색 차선과 하얀색 차선을 인식해서 바이너리 이미지로 바꿔서 출력해준다.
import rospy
import rospkg
import numpy as np
import cv2
import os

lower_white = np.array([200,200,0],dtype=np.uint8)
upper_white = np.array([255,255,255],dtype=np.uint8)
upper_yellow = np.array([30,100,100],dtype=np.uint8)
lower_yellow = np.array([90,255,255],dtype=np.uint8)

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class IMGPaser:
    def __init__(self):
        #이미지를 받아오는 섭스크라이버
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
    
    #이미지를 받아오는 콜백함수
    def callback(self, msg):
        try:
            #컴프레스드 된 이미지는 바이트 단위로 되어있고 그것을 인트로 변환
            np_arr = np.fromstring(msg.data, np.uint8)
            #인트로 변환한 이미지를 디코드 해준다.
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        #이미지를 HSV로 변환
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        #흰색과 노란색을 추출
        img_white_line  = cv2.inRange(img_hsv, lower_white, upper_white)
        img_yellow_line = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        img_line = cv2.bitwise_or(img_white_line, img_yellow_line)

        cv2.imshow("Image window", img_line)
        cv2.waitKey(1)
    
if __name__ == '__main__':
    rospy.init_node('line_detect', anonymous=True)
    camera = IMGPaser()
    rospy.spin()