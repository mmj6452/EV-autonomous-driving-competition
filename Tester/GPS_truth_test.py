#!/usr/bin/env python
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt
from morai_msgs.msg import GPSMessage
import os
from pyproj import Proj
from std_msgs.msg import Float32MultiArray


class pathMaker : # 실제 동작 -> "좌표 저장"
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True) #노드생성
        # /turtle1/pose 토픽 구독
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_status=False
        # 패키지 경로 로드 & 파일 쓰기 모드 (like 저장)
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path = '/home/ubuntu/catkin_ws/src/EV/path/path.txt'
        #full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                # turtle 위치 기록 (if 셧다운 아니면 반복해서 위치 기록)
                data='{0}\t{1}\n'.format(self.utm_x,self.utm_y)
                self.f.write(data)
                print("write : ", self.utm_x,self.utm_y)
        self.f.close()


    def gps_callback(self, gps_msg):
            self.is_gps_data = True
            longitude = gps_msg.longitude
            latitude = gps_msg.latitude
            utm_xy = self.proj_UTM(longitude, latitude)
            self.utm_x = utm_xy[0]
            self.utm_y = utm_xy[1]
            self.map_x = self.utm_x - gps_msg.eastOffset
            self.map_y = self.utm_y - gps_msg.northOffset
            data='{0}\t{1}\n'.format(self.utm_x,self.utm_y)
            self.f.write(data)
            print("write : ", self.utm_x,self.utm_y)
            
            os.system('clear')
            print("-------------------------------------")
            print(f"longitude : {gps_msg.longitude}")
            print(f"latitude : {gps_msg.latitude}")
            print()
            print(f"utm_x : {self.utm_x}")
            print(f"utm_y : {self.utm_y}")
            print(f"simulator map_x : {self.map_x}")
            print(f"simulator map_y : {self.map_y}")
            print("-------------------------------------")
            self.is_status=True

if __name__ == '__main__' :
    try:
        p_m=pathMaker("beginner_tutorials", "turtle_path") # pathMaker("패키지명","저장할 경로파일명")
        # Get_GPS.py 경로 : /KNU_EV/Code/src/EV/script/Get_GPS.py
    except rospy.ROSInternalException:
        pass
            