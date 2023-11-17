from std_msgs.msg import Bool, Float64, Int8
# from fiducial_msgs.msg import FiducialArray
from cv_bridge import CvBridge
# from sensor_msgs.msg import CompressedImage, LaserScan
from sensor_msgs.msg import Image , LaserScan #bag파일 topic

import cv2
import rospy
from BirdEyeView import *
from function import *
import numpy as np
import math, time

class Detection:
    def __init__(self):
        # houghLine_pub = rospy.Publisher('HoughLine', Int32MultiArray, queue_size=5)
        rospy.init_node('Detection')
        # rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.camera_callback) #ocam
        rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback) #bagfile확인용
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        self.pts = []
        self.cte_pub = rospy.Publisher('/LaneFollow', Float64, queue_size=1)
        self.rate = rospy.Rate(50)
        self.bridge = CvBridge()
        self.offset = 30  ##조향값들 수정 필요 
        self.middle = 0  ##
        self.gain = 0
        self.semaphore = 0
        self.count = 0
        self.pixelRate_km = (138-38)/(400-313) #픽셀과 실제 거리비율
        self.xThresh_km = 33
        self.yThresh_km = 313
        self.distance_km = 115 # 국민대 차선과 중심간의 거리
        self.lanewidth_km = 0.175
        self.error = 0
        self.gps_const = 0
        
    def sinho(self, data): ##신호등 검출한다면 여기에 추가
        if data.data == 1:
            self.semaphore = 1
            print('MARKER DETECTED')
            

    def lidar_callback(self, data):
        # resolution : 1285
        # increment : 0.28 deg
        self.lidar = data.ranges

    #ocam 카메라 콜백  no warpping ver
    def camera_callback(self, img):
        if self.gps_const == 0: #gps데이터값이 들어오지 않을때,
            try : 
                # img = self.bridge.compressed_imgmsg_to_cv2(img, 'bgr8') #ocam
                img = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")  #bag파일 확인용
                #################### 메세지 타입 변환 rosmsg -> opencv msg ###################
                resized_img = cv2.resize(img, dsize=(0,0), fx=0.5, fy=0.4, interpolation=cv2.INTER_LINEAR) 

                # cv2.imshow('rz_img', resized_img)  # 확인완료 
                cv2.waitKey(5)
                w = img.shape[1] #1280
                h = img.shape[0] #960
                # print(w,h)
            except : return
            
            #gray_warped = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   #warping일단 보류, 두가지 케이스로 테스트
            hsv = cv2.cvtColor(resized_img, cv2.COLOR_BGR2HSV)  # Convert BGR to HSV
            lower_yellow = np.array([15, 50, 50])  
            upper_yellow = np.array([33, 255, 255])  #컬러마스크는 되도록 건드리지 말것
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  
            yellow_img = cv2.bitwise_and(resized_img, resized_img, mask=yellow_mask)  
            cv2.imshow("yellow_img",yellow_img)
            blur_img = cv2.GaussianBlur(yellow_img,(5,5),0) 
            low_threshold = 30 #애는 밑에 threshold에서 얼마만큼 근접한지. 너무 낮추면 점같은 노이즈 생김
            high_threshold = 80 #애가 진짜 threshhold

            edges = cv2.Canny(blur_img, low_threshold, high_threshold)
            cv2.imshow("canny_img",edges)

            #################### hough line transform, 기울기 구하기 ###################
            #bagfile#
            mask , _shape = roi(edges)
            cv2.imshow("roi",mask)
            #ocam#

            ###try를 사용해서 초기에 line이 검출되지 않으면 작동이 안할 수도 있음. 
            ##pass구문 대신에 속도를 유지하는 방향으로 사용하는 것이 좋아보임
            try:      
                rho = 2
                theta = np.pi/180
                threshold = 90
                min_line_len = 50
                max_line_gap = 300     
                lines_img = hough_lines(mask,rho,theta,threshold,min_line_len,max_line_gap)
                cv2.imshow("lines",lines_img)
                lines = hough_lines2(mask,rho,theta,threshold,min_line_len,max_line_gap)


            except : pass
                
            #차선의 기울기를 이용해서 어느방향 차선인지 구분
            
            upperPt, lowerPt = getLines(resized_img,lines)
            if upperPt!=None and lowerPt!=None:
                # print(f' upper : {len(upperPt)}')
                # print(f' lower : {len(lowerPt)}')
                if len(upperPt)==1:
                    centerX = (upperPt[0][0]+lowerPt[0][0])//2
                    centerY = (upperPt[0][1]+lowerPt[0][1])//2
                    if (upperPt[0][1]-lowerPt[0][1])/(upperPt[0][0]-lowerPt[0][0])>0: # 오른쪽 차선
                        # goal = centerX - (self.pixelRate_sd * (centerY-self.yThresh_sd) + self.xThresh_sd)
                        goal = centerX - (self.pixelRate_km * (centerY-self.yThresh_km) + self.xThresh_km)
                        cv2.circle(resized_img, (int(goal), centerY), 10, [0,255,255])
                        cv2.circle(resized_img, (int(goal-70), centerY), 10, [0,0,255])
                        goal = goal-70
                        text=" RIGHT"    
                        org=(int(goal), centerY)
                        font=cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(resized_img,text,org,font,1,(255,0,0),2)
                        # error = (goal-w//2)*(self.lanewidth_sd/sel    f.distance_sd)
                        error = (goal-w//2)*(self.lanewidth_km/self.distance_km)
                        self.cte_pub.publish(error)
                        self.prev_goal = goal
                    else:
                        # goal = centerX + (self.pixelRate_sd * (centerY-self.yThresh_sd) + self.xThresh_sd)
                        goal = centerX + (self.pixelRate_km * (centerY-self.yThresh_km) + self.xThresh_km)
                        cv2.circle(img, (int(goal), centerY), 10, [0,255,255])
                        cv2.circle(img, (int(goal+70), centerY), 10, [0,0,255])
                        goal = goal+70
                        text="LEFT"
                        org=(int(goal), centerY)
                        font=cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(img,text,org,font,1,(255,0,0),2)
                        # error = (goal-w//2)*(self.lanewidth_sd/self.distance_sd)
                        error = (goal-w//2)*(self.lanewidth_km/self.distance_km)
                        self.cte_pub.publish(error)
                        self.prev_goal = goal

                elif len(upperPt)==2:
                    crossX, crossY = calculate_intersection(upperPt, lowerPt)
                    cv2.circle(resized_img, (crossX, crossY), 10, [0,255,255])
                    text="CROSS"    
                    org=(crossX, crossY)
                    font=cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(resized_img,text,org,font,1,(255,0,0),2)
                    # error = (crossX-w//2)*(self.lanewidth_sd/self.distance_sd)
                    error = (crossX-w//2)*(self.lanewidth_km/self.distance_km)
                    self.cte_pub.publish(error)

            else:
                print(f'upperPt : {upperPt}')
                # print(f' lower : {lowerPt}')
                if self.prev_goal is not None:
                    goal = self.prev_goal
                    print(goal)
            cv2.line(resized_img, (w//2,h), (w//2,h-30), color=[0,0,255], thickness=2)

            # 신호등 발견시 
            cnt = 0
            if self.semaphore == 1:
                if cnt==0:
                    print("AR")
                    cnt += 1
                self.markerFlag.publish(1)
                # 원본 이미지 복사
                result_image = img.copy()
                
                
                # 정지선을 원본 이미지에 그리기
                if lines is not None:
                    # print(f' lines :{lines}')
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        slope = (y2 - y1) / (x2 - x1) if x2 != x1 else math.inf
                        print(f'slope : {slope}')
                        if -0.3 <= slope <= 0.3:  # 기울기가 -0.1에서 0.1 사이인 경우 (수평한 선)
                            cv2.line(result_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            # stop point , 
                            stop_point_y = ((y1 + y2) // 2 ) + 105
                            print(f'stop_point_y : {stop_point_y}')
                            # 문제의 원인, roi상에서 선이 인식되는 범위에서 멈춰야됨
                            
                            # 정지선 발견, 1번만 발견해야 함
                            if stop_point_y >= 180 and self.rsSemaphore==False:# 정지 포인트일때
                                print("stopline")
                                self.rsSemaphore=True # 정지선 1번만 탐지하도록
                                rospy.sleep(5) # speed = 0
                                self.semaphore=2
                                self.markerFlag.publish(1)
                                print('speed : 1000')
                                break
                            elif stop_point_y >= 180 and self.rsSemaphore==True:
                                self.rsSemaphore=False
                                self.markerFlag.publish(2)
                                print('speed : 2000')

                            #self.semaphore = 0

                            # 정지선 통과한 뒤, 신호등 발견하면 다시 정지해야 하기 때문에, 
                            # self.semaphore를 0으로 돌려놔야 한다. 


if __name__ == '__main__':
    
    Detection()
    rospy.spin()
