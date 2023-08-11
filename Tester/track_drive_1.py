#!/usr/bin/env python
# -*- coding: utf-8 -*- 15
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업 라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 자이카 C모델 Ubuntu 20.04 + ROS Noetic
#=============================================
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 17  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 카메라 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
Blue = (0,255,0) # 파란색
Green = (0,255,0) # 녹색
Red = (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
stopline_num = 1 # 정지선 발견때마다 1씩 증가
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치

#=============================================
# 차선 인식 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30  # 카메라 FPS 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
ROI_START_ROW = 300  # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 380  # 차선을 찾을 ROT 영역의 끝 Row값
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = 40  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 

#=============================================
# 프로그램에서 사용할 이동평균필터 클래스
#=============================================
class MovingAverage:

    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
            
    def get_sample_count(self):
        return len(self.data)
        
    # 이동평균값을 구하는 함수
    def get_mavg(self):
        return float(sum(self.data)) / len(self.data)

    # 중앙값을 사용해서 이동평균값을 구하는 함수
    def get_mmed(self):
        return float(np.median(self.data))

    # 가중치를 적용하여 이동평균값을 구하는 함수        
    def get_wmavg(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])
        
#=============================================
# 초음파 8개의 거리정보에 대해서 이동평균필터를 적용하기 위한 선언
#=============================================
avg_count = 5  # 이동평균값을 계산할 데이터 묶음 갯수
ultra_mvavg = [MovingAverage(avg_count) for i in range(8)]

#=============================================
# 콜백함수 - USB 전방카메라 토픽을 처리하는 콜백함수.
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 콜백함수 - 초음파 토픽을 처리하는 콜백함수.
#=============================================
def ultra_callback(data):
    global ultra_msg, ultra_data
    #ultra_msg = data.data
    ultra_data = data.data

    # 이동평균필터를 적용해서 튀는 값을 제거해서 ultra_msg_ft에 담기
    for i in range(8):
        ultra_mvavg[i].add_sample(float(ultra_data[i]))
        
    ultra_list = [int(ultra_mvavg[i].get_mmed()) for i in range(8)]
    ultra_msg = tuple(ultra_list)

#=============================================
# 모터 토픽을 발행하는 함수.  
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수.  
# 입력으로 시간(초)를 받아 그 시간동안 속도=0 토픽을 모터로 보냄
#=============================================
def stop_car(sleep_sec):
    for i in range(sleep_sec*100): 
        drive(angle=new_angle, speed=0)
        time.sleep(0.01)
    
#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 핸들 조정함.
#=============================================
def sonic_drive():
    global new_angle, new_speed

    # 앞쪽 가까이에 장애물이 있으면 차량 멈춤
    if (min(ultra_msg[1], ultra_msg[2], ultra_msg[3]) < 5):
        new_angle = new_angle
        new_speed = 0
        print("Car Brake, Stop! : ", ultra_msg)

    # 왼쪽이 오른쪽보다 조금 멀리 있으면 있으면 작은 각도로 좌회전 주행
    elif (ultra_msg[0]-ultra_msg[4] > 10) or (ultra_msg[1]-ultra_msg[3] > 10):
        new_angle = -15
        new_speed = Fix_Speed
        print("Turn left1 : ", ultra_msg)
        
    # 왼쪽이 오른쪽보다 많이 멀리 있으면 있으면 큰 각도로 좌회전 주행
    elif (ultra_msg[0]-ultra_msg[4] > 20) or (ultra_msg[1]-ultra_msg[3] > 20):
        new_angle = -20
        new_speed = Fix_Speed
        print("Turn left1 : ", ultra_msg)
    
    # 오른쪽이 왼쪽보다 조금 멀리 있으면 있으면 작은 각도로 우회전 주행
    elif (ultra_msg[4]-ultra_msg[0] > 10) or (ultra_msg[3]-ultra_msg[1] > 10):
        new_angle = 15
        new_speed = Fix_Speed
        print("Turn right1 : ", ultra_msg)
   
    # 오른쪽이 왼쪽보다 많이 멀리 있으면 있으면 큰 각도로 우회전 주행
    elif (ultra_msg[4]-ultra_msg[0] > 20) or (ultra_msg[3]-ultra_msg[1] > 20):
        new_angle = 20
        new_speed = Fix_Speed
        print("Turn right1 : ", ultra_msg)
   
   # 위 조건에 해당하지 않는 경우라면 (오른쪽과 왼쪽이 비슷한 경우) 똑바로 직진 주행
    else:
        new_angle = 0
        new_speed = Fix_Speed
        print("Go Straight : ", ultra_msg)

    # 모터에 주행명령 토픽을 보낸다
    drive(new_angle, new_speed)

#=============================================
# 카메라 이미지를 영상처리하여 
# 정지선이 있는지 체크하고 True 또는 False 값을 반환.
#=============================================
def check_stopline():
    global stopline_num

    # 원본 영상을 화면에 표시
    #cv2.imshow("Original Image", image)
    
    # image(원본이미지)의 특정영역(ROI Area)을 잘라내기
    roi_img = image[300:480, 0:640]
    cv2.imshow("ROI Image", roi_img)

    # HSV 포맷으로 변환하고 V채널에 대해 범위를 정해서 흑백이진화 이미지로 변환
    hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV) 
    upper_white = np.array([255, 255, 255])
    lower_white = np.array([0, 0, 180])
    binary_img = cv2.inRange(hsv_image, lower_white, upper_white)
    #cv2.imshow("Black&White Binary Image", binary_img)

    # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
    stopline_check_img = binary_img[100:120, 200:440] 
    #cv2.imshow("Stopline Check Image", stopline_check_img)
    
    # 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
    img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(img, (200,100),(440,120),Green,3)
    cv2.imshow('Stopline Check', img)
    cv2.waitKey(1)
    
    # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
    stopline_count = cv2.countNonZero(stopline_check_img)
    
    # 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
    if stopline_count > 2500:
        print("Stopline Found...! -", stopline_num)
        stopline_num = stopline_num + 1
        cv2.destroyWindow("ROI Image")
        return True
    
    else:
        return False
        
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image, img_ready 
    global new_angle, new_speed

    SENSOR_DRIVE = 1
    TRAFFIC_SIGN = 2
    LANE_DRIVE = 3
    AR_DRIVE = 4
    OBJECT_DETECT = 5
    FINISH = 9
    drive_mode = SENSOR_DRIVE
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)

    #=========================================
    # 첫번째 토픽이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")

    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # ======================================
        # 초음파 센서로 주행합니다.
        # 정지선이 보이면 OBJECT_DETECTION 모드로 변경합니다.
        # ======================================
        while drive_mode == SENSOR_DRIVE:
            sonic_drive()
            result = check_stopline()
             
            if (result == True):
                stop_car(1)
                drive_mode = FINISH 
                print ("----- Finish driving... -----")
                
        # ======================================
        # 주행을 끝냅니다. 
        # ======================================
        if drive_mode == FINISH:        
            # 차량을 정지시키고 모든 작업을 끝냅니다.
            stop_car(1)  
            time.sleep(2)
            print ("----- Bye~! -----")
            return            

#=============================================
# 메인함수 호툴
# start() 함수가 실질적인 메인함수임. 
#=============================================
if __name__ == '__main__':
    start()
