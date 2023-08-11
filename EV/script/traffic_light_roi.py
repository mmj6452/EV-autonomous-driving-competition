#화면상에서 신호등이 보이는 영역만 추출해내고 붉은색 또는 노란색을 추출하여 신호등의 상태를 확인한다. 

#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
'''
lower_red = np.array([0, 200, 200], dtype=np.uint8)
upper_red = np.array([30, 255, 255], dtype=np.uint8)

lower_yellow = np.array([30, 150, 150], dtype=np.uint8)
upper_yellow = np.array([50, 255, 255], dtype=np.uint8)
'''

lower_red = (0, 0, 200)
upper_red = (100, 100, 255)
#lower_yellow = (0, 200, 200)
#upper_yellow = (100, 255, 255)

class IMGParser:

    def __init__(self):
        # 이미지를 받아오는 섭스크라이버
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        #print("you need to find the right value : line 19 ~ 22")
        ###마스크의 크기를 정하는 부분 안에 숫자는 교체 해야함
        self.crop_pts = np.array(
            [[
                [220,200],
                [220,0],
                [420,0],
                [420,200]
            ]]
        )

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]

        if len(img.shape) == 3:

            # image shape : [h, w, 3]

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:

            # binarized image or grayscale image : [h, w]

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask
    def callback(self, msg):
        try:
            # 컴프레스드 된 이미지는 바이트 단위로 되어있고 그것을 인트로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)
            # 인트로 변환한 이미지를 디코드 해준다.
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        # 이미지를 마스크 처리 한 이미지로 변환

        self.mask = self.mask_roi(img_bgr)


        if len(self.mask.shape)==3:
            img_concat = np.concatenate([img_bgr, self.mask], axis=1)

        else:
            img_concat = np.concatenate([img_bgr, cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)], axis=1)

        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)
        # 이미지를 hsv로 변환

        #img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        #img_hsv = cv2.cvtColor(self.mask, cv2.COLOR_BGR2RGB)

        # 흰색과 노란색을 추출 ###신호등으로 변경할 부분 신호등의 주황색은 R과 G값이 검출될 것으로 추정
        img_red_light = cv2.inRange(self.mask, lower_red, upper_red)
        #img_yellow_light = cv2.inRange(self.mask, lower_yellow, upper_yellow)
        #img_traffic_light_stop = img_red_light
        #img_traffic_light_stop = cv2.bitwise_or(img_red_light, img_yellow_light)
        if cv2.countNonZero(img_red_light) > 0:
            reddot= cv2.countNonZero
        else:
            reddot=0
        
        cv2.imshow("traffic light Image windowr", img_red_light)
        #cv2.imshow("traffic light Image windowy", img_yellow_light)
        #cv2.imshow("traffic light Image windows", img_traffic_light_stop)  ###여기서 라인을 신호로 변경
        cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
