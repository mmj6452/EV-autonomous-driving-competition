import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import GPSMessage
from pyproj import Proj
import os

WIDTH = 640
HEIGHT = 480
ROI_START_ROW = 200
ROI_END_ROW = 350
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW
HANDLE_ANGLE = 0.007



L_ROW = 40
CAM_FPS = 30
View_Center = WIDTH//2
Fix_Speed = 17
new_angle = 0
new_speed = Fix_Speed
stopline_num = 1
boundary_points = [
    (180, 180),
    (182, 186),
    (166, 183),
    (168, 190),
]

class line_drive:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.is_gps_in_boundary = True
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.rate = rospy.Rate(30)
        self.cmd = CtrlCmd()
        self.cmd.longlCmdType = 2
        self.cmd.velocity = 0
        self.cmd.steering = [0, 0]
        self.boundary_points = [
                            (180, 180),
                            (182, 186),
                            (166, 183),
                            (168, 190),
                            ]

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_ready = True
            if self.is_gps_in_boundary == False:
                return
            speed,angle = self.lane_drive(self.img_bgr, self.img_ready)
            speed = float(speed)
            angle = float(angle)*HANDLE_ANGLE
            print("speed: ", speed, "angle: ", angle)
            if(angle > 0):
                angle = angle*0.85
            self.cmd.steering = -angle
            self.cmd.velocity = speed
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()
        except CvBridgeError as e:
            print(e)

    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        longitude = gps_msg.longitude
        latitude = gps_msg.latitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        self.map_x = utm_x - gps_msg.eastOffset
        self.map_y = utm_y - gps_msg.northOffset
        
        os.system('clear')
        if self.is_within_boundary((self.map_x,self.map_y), self.boundary_points):
            self.is_gps_in_boundary = False
            print("out of boundary")
            return
        print(f"simulator self.map_x : {self.map_x}")
        print(f"simulator self.map_y : {self.map_y}")
        print("boundary : ", self.is_gps_in_boundary )
    #=============================================
    # 카메라 영상 이미지에서 차선을 찾아
    # 차선을 벗어나지 않으며 주행하도록 핸들 조정함.
    #=============================================
    def lane_drive(self,image,img_ready):
        prev_x_left = 0
        prev_x_right = WIDTH

        while img_ready == False:
            continue

        img = image.copy() # 이미지처리를 위한 카메라 원본이미지 저장
        display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장
        img_ready = False  # 카메라 토픽이 도착하면 콜백함수 안에서 True로 바뀜

        #=========================================
        # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
        # 블러링 처리를 통해 노이즈를 제거한 후에 (약간 뿌옇게, 부드럽게)
        # Canny 변환을 통해 외곽선 이미지로 만들기
        #=========================================
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), 60, 75)
    
        # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
        roi_img = img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
        
        # edge_img의 특정영역(ROI Area)을 잘라내기
        roi_edge_img = edge_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH]
        
        cv2.imshow("roi edge img", roi_edge_img)

        # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
        all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180,50,50,20)
        
        if all_lines is None:
            return
            
        line_draw_img = roi_img.copy()
        
        #=========================================
        # 선분들의 기울기 값을 각각 모두 구한 후에 리스트에 담음. 
        # 기울기의 절대값이 너무 작은 경우 (수평선에 가까운 경우)
        # 해당 선분을 빼고 담음. 
        #=========================================
        slopes = []
        filtered_lines = []

        for line in all_lines:
            x1, y1, x2, y2 = line[0]

            if (x2 == x1):
                slope = 1000.0
            else:
                slope = float(y2-y1) / float(x2-x1)
        
            if 0.2 < abs(slope):
                slopes.append(slope)
                filtered_lines.append(line[0])

        # print("Number of lines after slope filtering : %d" % len(filtered_lines))

        if len(filtered_lines) == 0:
            return

        #=========================================
        # 왼쪽 차선에 해당하는 선분과 오른쪽 차선에 해당하는 선분을 구분하여 
        # 각각 별도의 리스트에 담음.
        #=========================================
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = filtered_lines[j]
            slope = slopes[j]

            x1,y1, x2,y2 = Line

            # 기울기 값이 음수이고 화면의 왼쪽에 있으면 왼쪽 차선으로 분류함
            # 기준이 되는 X좌표값 = (화면중심값 - Margin값)
            Margin = 0
            
            if (slope < 0) and (x2 < WIDTH/2-Margin):
                left_lines.append(Line.tolist())

            # 기울기 값이 양수이고 화면의 오른쪽에 있으면 오른쪽 차선으로 분류함
            # 기준이 되는 X좌표값 = (화면중심값 + Margin값)
            elif (slope > 0) and (x1 > WIDTH/2+Margin):
                right_lines.append(Line.tolist())

        # print("Number of left lines : %d" % len(left_lines))
        # print("Number of right lines : %d" % len(right_lines))

        # 디버깅을 위해 차선과 관련된 직선과 선분을 그리기 위한 도화지 준비
        line_draw_img = roi_img.copy()
        
        # 왼쪽 차선에 해당하는 선분은 빨간색으로 표시
        for line in left_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,0,255), 2)

        # 오른쪽 차선에 해당하는 선분은 노란색으로 표시
        for line in right_lines:
            x1,y1, x2,y2 = line
            cv2.line(line_draw_img, (x1,y1), (x2,y2), (0,255,255), 2)

        #=========================================
        # 왼쪽/오른쪽 차선에 해당하는 선분들의 데이터를 적절히 처리해서 
        # 왼쪽차선의 대표직선과 오른쪽차선의 대표직선을 각각 구함.
        # 기울기와 Y절편값으로 표현되는 아래와 같은 직선의 방적식을 사용함.
        # (직선의 방정식) y = mx + b (m은 기울기, b는 Y절편)
        #=========================================

        # 왼쪽 차선을 표시하는 대표직선을 구함        
        m_left, b_left = 0.0, 0.0
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

        # 왼쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(left_lines)
        if size != 0:
            for line in left_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0                
                
            x_avg = x_sum / (size*2)
            y_avg = y_sum / (size*2)
            m_left = m_sum / size
            b_left = y_avg - m_left * x_avg

            if m_left != 0.0:
                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = 0
                # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                x1 = int((0.0 - b_left) / m_left)

                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = ROI_HEIGHT
                # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                x2 = int((ROI_HEIGHT - b_left) / m_left)

                # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
                cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

        # 오른쪽 차선을 표시하는 대표직선을 구함      
        m_right, b_right = 0.0, 0.0
        x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

        # 오른쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
        size = len(right_lines)
        if size != 0:
            for line in right_lines:
                x1, y1, x2, y2 = line
                x_sum += x1 + x2
                y_sum += y1 + y2
                if(x2 != x1):
                    m_sum += float(y2-y1)/float(x2-x1)
                else:
                    m_sum += 0     
        
            x_avg = x_sum / (size*2)
            y_avg = y_sum / (size*2)
            m_right = m_sum / size
            b_right = y_avg - m_right * x_avg

            if m_right != 0.0:
                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = 0
                # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                x1 = int((0.0 - b_right) / m_right)

                #=========================================
                # (직선 #1) y = mx + b 
                # (직선 #2) y = ROI_HEIGHT
                # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                x2 = int((ROI_HEIGHT - b_right) / m_right)

                # 두 교점, (x1,0)과 (x2, ROI_HEIGHT)를 잇는 선을 그림
                cv2.line(line_draw_img, (x1,0), (x2,ROI_HEIGHT), (255,0,0), 2)

        #=========================================
        # 차선의 위치를 찾기 위한 기준선(수평선)은 아래와 같음.
        # (직선의 방정식) y = L_ROW 
        # 위에서 구한 2개의 대표직선, 
        # (직선의 방정식) y = (m_left)x + (b_left)
        # (직선의 방정식) y = (m_right)x + (b_right)
        # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
        #=========================================

        #=========================================        
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
        #=========================================
        if m_left == 0.0:
            x_left = prev_x_left
    
        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_left)x + (b_left)
        #=========================================
        else:
            x_left = int((L_ROW - b_left) / m_left)
                            
        #=========================================
        # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
        # 이 경우에는 왼쪽 차선보다 예상되는 값만큼 오른쪽으로 세팅함 
        #=========================================
        if m_right == 0.0:
            x_right = x_left + 500

        #=========================================
        # 아래 2개 직선의 교점을 구함
        # (직선의 방정식) y = L_ROW  
        # (직선의 방정식) y = (m_right)x + (b_right)
        #=========================================
        else:
            x_right = int((L_ROW - b_right) / m_right)
            
        prev_x_left = x_left
        prev_x_right = x_right

        # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치를 구함

        x_midpoint = (x_left + x_right) // 2 

        # 화면의 중앙지점(=카메라 이미지의 중앙지점)을 구함
        view_center = WIDTH//2
    
        #=========================================
        # 디버깅용 이미지 그리기
        # (1) 수평선 그리기 (직선의 방정식) y = L_ROW 
        # (2) 수평선과 왼쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (3) 수평선과 오른쪽 대표직선과의 교점 위치에 작은 녹색 사각형 그리기 
        # (4) 왼쪽 교점과 오른쪽 교점의 중점 위치에 작은 파란색 사각형 그리기
        # (5) 화면의 중앙점 위치에 작은 빨간색 사각형 그리기 
        #=========================================
        cv2.line(line_draw_img, (0,L_ROW), (WIDTH,L_ROW), (0,255,255), 2)
        cv2.rectangle(line_draw_img, (x_left-5,L_ROW-5), (x_left+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_right-5,L_ROW-5), (x_right+5,L_ROW+5), (0,255,0), 4)
        cv2.rectangle(line_draw_img, (x_midpoint-5,L_ROW-5), (x_midpoint+5,L_ROW+5), (255,0,0), 4)
        cv2.rectangle(line_draw_img, (view_center-5,L_ROW-5), (view_center+5,L_ROW+5), (0,0,255), 4)

        # 위 이미지를 디버깅용 display_img에 overwrite해서 화면에 디스플레이 함
        display_img[ROI_START_ROW:ROI_END_ROW, 0:WIDTH] = line_draw_img
        cv2.imshow("Lanes positions", display_img)
        cv2.waitKey(1)

        # ====================================
        # 핸들을 얼마나 꺾을지 결정 
        # ====================================
        angle = (x_midpoint-view_center) // 2

        # ====================================
        # 주행 속도를 결정  
        # ====================================
        speed = 20

        # ====================================
        # 모터쪽으로 토픽을 전송  
        # ====================================
        return speed, angle

    def is_within_boundary(self,current_location, boundary_points):
        def euclidean_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        for i in range(len(boundary_points)):
            p1 = boundary_points[i]
            p2 = boundary_points[(i + 1) % len(boundary_points)]

            distance1 = euclidean_distance(current_location, p1)
            distance2 = euclidean_distance(current_location, p2)
            distance3 = euclidean_distance(p1, p2)

            if distance1 + distance2 <= distance3 + 1e-6:
                return True
        return False

if __name__ == '__main__':
    rospy.init_node('lane_drive', anonymous=True)
    camera = line_drive()
    rospy.spin()