#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os

from pyproj import Proj
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Vector3
from morai_msgs.msg import GPSMessage
import math



class GPS_to_UTM:
    before_x = 0
    before_y = 0
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)
        self.Speed_pub = rospy.Publisher("/Speed", Vector3, queue_size=1)
        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rospy.spin()


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        longitude = gps_msg.longitude
        latitude = gps_msg.latitude
        self.utm_xy = self.proj_UTM(longitude, latitude)
        self.utm_x = self.utm_xy[0]
        self.utm_y = self.utm_xy[1]
        map_x = self.utm_x - gps_msg.eastOffset
        map_y = self.utm_y - gps_msg.northOffset
        speed_X , speed_Y , speed = self.Speed_cal()
        self.Speed_pub.publish( speed_X , speed_Y , speed )
        
        os.system('clear')
        print("-------------------------------------")
        print(f"self.utm_x : {self.utm_x}")
        print(f"self.utm_y : {self.utm_y}")
        print(f"simulator map_x : {map_x}")
        print(f"simulator map_y : {map_y}")
        print(f"Speed : {self.Speed}")
        print(f"Speed_x : {self.Speed_x}")
        print(f"Speed_y : {self.Speed_y}")
        print("-------------------------------------")
    
    def Speed_cal(self):
        self.Speed = (math.sqrt((self.before_x-self.utm_x)**2 + (self.before_y-self.utm_y)**2))/0.03
        self.Speed_x = (math.sqrt((self.utm_x - self.before_x)**2))/0.03
        self.Speed_y = (math.sqrt((self.utm_y - self.before_y)**2))/0.03
        self.Speed = self.Speed*3.3
        self.before_x = self.utm_x
        self.before_y = self.utm_y
        return self.Speed_x , self.Speed_y , self.Speed


if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass
