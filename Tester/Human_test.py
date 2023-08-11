#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import cv2
import os

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class Get_Camera:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            is_human = self.Hunmen_detect(self.img_bgr)
            if is_human:
                print("Human detected")
        except CvBridgeError as e:
            print(e)
    
    def Hunmen_detect(self,img_bgr):
        body_cascade = cv2.CascadeClassifier('/home/ubuntu/catkin_ws/src/Tester/src/haarcascade_fullbody.xml')
        is_human = False
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)
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
    rospy.init_node('Get_img', anonymous=True)
    camera = Get_Camera()
    rospy.spin()
