#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os

#좌표계 변환 라이브러리
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage


#GPS를 UTM좌표로 변환해주는 클래스
class GPS_to_UTM:
    def __init__(self):
        #노드생성
        rospy.init_node('GPS_to_UTM', anonymous=True)
        #섭스크라이버 설정
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        #GPS데이터를 UTM으로 변경 존 52는 한국의 존
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rospy.spin()


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        #경도 읽어오기
        longitude = gps_msg.longitude
        #위도 읽어오기
        latitude = gps_msg.latitude
        #좌표계변환
        utm_xy = self.proj_UTM(longitude, latitude)
        #X좌표Y좌표에 각각 저장
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        #Offset에서 이동거리를 빼서 위치를 표현
        map_x = utm_x - gps_msg.eastOffset
        map_y = utm_y - gps_msg.northOffset
        
        os.system('clear')
        print("-------------------------------------")
        print(f"longitude : {gps_msg.longitude}")
        print(f"latitude : {gps_msg.latitude}")
        print()
        print(f"utm_x : {utm_x}")
        print(f"utm_y : {utm_y}")
        print(f"simulator map_x : {map_x}")
        print(f"simulator map_y : {map_y}")
        print("-------------------------------------")


if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass