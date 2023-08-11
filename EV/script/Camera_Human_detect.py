#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 카메라에서 사람을 인식하는 코드
# 카메라에서 사람을 인식하면 is_human = True로 바꾸고
# is_human이 True이면 사람이 인식되었다고 출력한다.
# 주의사항으로 harrcascade_fullbody.xml 파일이 있는지 확인해야한다.

import rospy
import rospkg
import numpy as np
import cv2
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
body_cascade = cv2.CascadeClassifier('/home/ubuntu/catkin_ws/src/EV/src/haarcascade_fullbody.xml')
is_human = False

class Get_Camera:
    def __init__(self):
        #이미지를 받아오는 섭스크라이버
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        is_human = False
    
    #이미지를 받아오는 콜백함수
    def callback(self, msg):
        try:
            #컴프레스드 된 이미지는 바이트 단위로 되어있고 그것을 인트로 변환
            np_arr = np.fromstring(msg.data, np.uint8)
            #인트로 변환한 이미지를 디코드 해준다.
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        self.Hunmen_detect(img_bgr)

    
    def Hunmen_detect(self,img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        # body detection.
        bodies = body_cascade.detectMultiScale(gray, 1.1, 4)
        if len(bodies) > 0:
            is_human = True
            # Draw a rectangle around the bodies
        for (x, y, w, h) in bodies:
            cv2.rectangle(img_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)
        return is_human
    
if __name__ == '__main__':
    rospy.init_node('line_detect', anonymous=True)
    camera = Get_Camera()
    rospy.spin()