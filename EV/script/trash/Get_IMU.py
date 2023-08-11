#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import tf
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi


class IMUParser:
    def __init__(self):
        #노드생성
        rospy.init_node('imu', anonymous=True)
        #섭스크라이버 생성
        self.image_sub = rospy.Subscriber("/imu", Imu, self.callback)

        self.br = tf.TransformBroadcaster()
        rospy.spin()

    def callback(self,data):
        #쿼터니엄값을 읽어옴
        quaternion=(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        #오일러값으로 변경
        roll,pitch,yaw = euler_from_quaternion(quaternion)
        roll_deg=roll/pi*180
        pitch_deg=pitch/pi*180
        yaw_deg=yaw/pi*180
        #터미널 상으로 출력
        os.system('clear')
        print("-------------------------------------")
        print(" Roll  (deg) = ",roll_deg)
        print(" Pitch (deg) = ",pitch_deg)
        print(" Yaw   (deg) = ",yaw_deg)
        print("-------------------------------------")
        self.prev_time=rospy.get_rostime()

if __name__ == '__main__' :
    try:
        imu_parser = IMUParser()
    except rospy.ROSInterruptException:
        pass
