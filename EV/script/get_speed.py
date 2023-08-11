#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import tf
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
import numpy as np

dt = 0.01

class Speed_calculator:
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
        self.roll_deg=roll/pi*180
        self.pitch_deg=pitch/pi*180
        self.yaw_deg=yaw/pi*180
        #터미널 상으로 출력
        os.system('clear')
        print("-------------------------------------")
        print(" Roll  (deg) = ",self.roll_deg)
        print(" Pitch (deg) = ",self.pitch_deg)
        print(" Yaw   (deg) = ",self.yaw_deg)
        print("-------------------------------------")
        self.prev_time=rospy.get_rostime()
        linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        eualer_angles = np.array([self.roll_deg, self.pitch_deg, self.yaw_deg])
        velocity = self.acceleration_to_velocity(linear_acceleration, eualer_angles, dt)
        print("velocity : ", velocity)
        return velocity

    
    def acceleration_to_velocity(self,linear_acceleration, euler_angles, dt):
        # 회전 행렬을 이용하여 가속도 데이터를 오일러 각에 맞게 변환합니다.
        R = np.array([
            [np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0],
            [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0],
            [0, 0, 1]
        ])
        rotated_acceleration = np.dot(R, linear_acceleration)

        # 변환된 가속도를 적분하여 속도를 구합니다.
        velocity = np.zeros(3)
        velocity[0] = rotated_acceleration[0] * dt
        velocity[1] = rotated_acceleration[1] * dt
        velocity[2] = rotated_acceleration[2] * dt
        return velocity

if __name__ == '__main__' :
    try:
        Speed_calculator = Speed_calculator()
    except rospy.ROSInterruptException:
        pass
