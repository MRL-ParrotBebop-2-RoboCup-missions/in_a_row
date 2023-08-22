#!/usr/bin/env python
from __future__ import print_function

# import roslib
# roslib.load_manifest('my_project')
import sys
import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Int16  # For error/angle plot publishing
from sensor_msgs.msg import Image
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from cv_bridge import CvBridge, CvBridgeError
import math
from pid import PID

class line_follower:

    def __init__(self):
        self.pub_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.pub_camdown = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
        # self.pub_roll_error = rospy.Publisher('/bebop/roll_error', Int16, queue_size=1)
        # self.pub_yaw_error = rospy.Publisher('/bebop/yaw_error', Int16, queue_size=1)
        self.bridge = CvBridge()
        
        self.battery_sub = rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged',
                                            CommonCommonStateBatteryStateChanged, self.battery)
        
        # start HSV #########################################################################################
        self.lower_start, self.upper_start = np.array([15, 196, 194]) , np.array([149, 255, 255])
        # Line HSV  #########################################################################################
        self.lower_line, self.upper_line  = np.array([20, 164, 140]) , np.array([166, 255, 255])

        self.battery = 0               
        self.yawE = 0
        self.rollE = 0

        self.Ppitch = 0
        self.Ipitch = 0
        self.Dpitch = 0
        # mrl
        self.Proll = 0.0001395
        self.Iroll = 0.00002393
        self.Droll = 0
        # # sasan
        # self.Proll = 0.00025
        # self.Iroll = 0.0004
        # self.Droll = 0.0

        self.Pyaw = 0.01
        self.Iyaw = 0.001
        self.Dyaw = 0

        self.pid_roll = PID(self.Proll, self.Droll, self.Iroll, -0.1, 0.1, -0.1, 0.1) 
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.5, 0.5, -0.1, 0.1) 

        self.twist = Twist()              
        self.tune_flag = False
        self.autonomous_flag = False
        self.stateFlag = False
        self.starter_drift_flag = False
        self.counter_zoom = 0
        self.counter_drift = 0
        self.contour_counter = 0
        self.turn_toggle = True
        self.movement_flag = 'start' #start, line, stop  without capitals
        self.line_finished_flag = False
        self.go_to_start_toggle = False
        self.start_rope_toggle = True
        self.start_line_toggle = True
        self.camera_angle = -50
        self.camera_down_start_time = time.time()
        self.was_line = False
    # Get the battery percentage
    def battery(self, data):
        self.battery = data.percent


    # Tilt down the camera in -75 degrees
    def camera_decrease(self):
        if  time.time() -  self.camera_down_start_time > 1:
            self.camera_angle = self.camera_angle - 3
            self.cam_down(self.camera_angle)
            self.camera_down_start_time = time.time()

    def cam_down(self, angle):
        cam = Twist()
        cam.angular.y = angle
        # print(angle, '111111111111111111111')
        self.pub_camdown.publish(cam)

    def keyboard_function(self, key):
        
        if  key == 27:  # close on ESC key
                cv2.destroyAllWindows()
                rospy.signal_shutdown('interrupt')   
                exit()  

        if key == ord('p'):
            self.tune_flag = 'p'        
        elif key == ord('r'):
            self.tune_flag = 'r'        
        elif key == ord('y'):
            self.tune_flag = 'y'       

        if key == ord('o'):
            self.autonomous_flag = not self.autonomous_flag    
        if key == ord('1'):            
            self.movement_flag = "start"            
        if key == ord('2'):
            self.movement_flag = "line"
        if key == ord('3'):
           
            self.movement_flag = "stop"
                                 
            
        if self.tune_flag == 'p':
            if key == ord('q')  :
                self.Ppitch = round(self.Ppitch + 0.00001, 6)
            elif key == ord('a')  :
                self.Ppitch = round(self.Ppitch - 0.00001, 6)
                
            if key == ord('w')  :
                self.Ipitch = round(self.Ipitch + 0.001, 4)
            elif key == ord('s')  :
                self.Ipitch = round(self.Ipitch - 0.001, 4)
                
            if key == ord('e')  :
                self.Dpitch = round(self.Dpitch + 0.0001, 4)
            elif key == ord('d')  :
                self.Dpitch = round(self.Dpitch - 0.0001, 4)
            elif key == ord(' ')  :
                self.Ppitch, self.Ipitch, self.Dpitch = 0, 0, 0
            

        elif self.tune_flag == 'r':
            if key == ord('q') :
                self.Proll = round(self.Proll + 0.00001, 6)
            elif key == ord('a') :
                self.Proll = round(self.Proll - 0.00001, 6)            

            if key == ord('w') :
                self.Iroll = round(self.Iroll + 0.0001, 4)
            elif key == ord('s') :
                self.Iroll = round(self.Iroll - 0.0001, 4)
            
            if key == ord('e') :
                self.Droll = round(self.Droll + 0.000000001, 10)
            elif key == ord('d') :
                self.Droll = round(self.Droll - 0.000000001, 10)            
            if key == ord(' ') :
                self.Proll, self.Iroll, self.Droll = 0, 0, 0
        
        elif self.tune_flag == 'y':
            if key == ord('q') :
                self.Pyaw = round(self.Pyaw + 0.001, 4)
            elif key == ord('a') :
                self.Pyaw = round(self.Pyaw - 0.001, 4)            

            if key == ord('w') :
                self.Iyaw = round(self.Iyaw + 0.001, 4)
            elif key == ord('s') :
                self.Iyaw = round(self.Iyaw - 0.001, 4)
            
            if key == ord('e') :
                self.Dyaw = round(self.Dyaw + 0.001, 4)
            elif key == ord('d') :
                self.Dyaw = round(self.Dyaw - 0.001, 4)            
            if key == ord(' ') :
                self.Pyaw, self.Iyaw, self.Dyaw = 0, 0, 0

    def controller(self, yaw_error, roll_error):

        self.pid_roll = PID(self.Proll, self.Droll, self.Iroll, -0.3, 0.3, -0.1, 0.1) 
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.15, 0.15, -0.1, 0.1) 
        
        yaw_treshold = 10
        roll_treshold = 75
        if self.starter_drift_flag:
            self.twist.linear.x = 0.09
              
        
        if abs(roll_error) <= roll_treshold and abs(yaw_error)<= yaw_treshold:            
            if self.start_rope_toggle:
                self.twist.linear.x =  0.02 #sasan
                # self.twist.linear.x =  0.01 #mrl
                self.twist.linear.y = 0.0
                self.twist.angular.z = 0.0  
            else:
                self.twist.linear.x =  0.09 #sasan
                # self.twist.linear.x =  0.07 #mrl
                self.twist.linear.y = 0.0
                self.twist.angular.z = 0.0         
                self.stateFlag = 'pitch'
                
        if abs(roll_error) > roll_treshold:
            # self.twist.linear.x = 0 #abs(self.pid_roll.update(roll_error)) * 0.9 for drift of roll ratio rate 
            #?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
            self.twist.linear.x = 0.0008            
            self.twist.linear.y = - self.pid_roll.update(roll_error)
            self.twist.angular.z = 0.0                
            self.stateFlag = 'roll'
            
        if abs(yaw_error) > 5 and  abs(roll_error) < 200:
            self.twist.linear.x = 0 #0.004 
            self.twist.linear.y = 0.0
            self.twist.angular.z = -self.pid_yaw.update(yaw_error)                  
            self.stateFlag='yaw'

        if self.autonomous_flag:
            self.pub_vel.publish(self.twist)
        

    def start_detect(self, cv_image):        
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)     
        start_mask = cv2.inRange(cv_image_hsv, self.lower_start, self.upper_start)       
        start_mask = cv2.dilate(start_mask, np.ones((5, 5), np.uint8), iterations=9)
        start_mask = cv2.erode(start_mask, np.ones((5, 5), np.uint8), iterations=5)       
        start_mask = cv2.morphologyEx(start_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8), iterations=1)
         
        contours_blk, _ = cv2.findContours(start_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)       
        contours_blk.sort(key=cv2.minAreaRect)

        if self.turn_toggle:
            turn_twist = Twist()
            turn_twist.angular.z = 0.1 
            if self.autonomous_flag:
                self.pub_vel.publish(turn_twist)

        if len(contours_blk) > 0 :           
            self.was_line = 1            
            blackbox = cv2.minAreaRect(contours_blk[-1])                        
            setpoint = cv_image.shape[1] / 2 # w/2
            
            center = (int(blackbox[0][0]),int(blackbox[0][1]))              
            rollE = int(center[0] - setpoint)
                      
            self.contour_counter = self.contour_counter + 1
            # 0.8 w for turning left .................................
            if 0.8 * cv_image.shape[1] - center[0] < 20 and self.contour_counter > 10 and  self.go_to_start_toggle==False:      
                print('go to start toggle')          
                self.contour_counter = 0  
                self.go_to_start_toggle = True  
                self.turn_toggle = False           
            
            if self.go_to_start_toggle: 
                if 0.3 * cv_image.shape[0] - center[1] > 1:
                    print('counter_drift', self.counter_drift)
                    self.counter_drift = self.counter_drift +1   # a counter for first time seeing start sign and before zoom for canceling roll drift
                    print('counter_drift')
                    if self.counter_drift > 4: #low pass filter
                        self.counter_drift = 0
                        self.starter_drift_flag = True
                        print('                        self.starter_drift_flag  ', True )
                else:
                    self.starter_drift_flag = False
                    self.twist = Twist()
                            
                self.controller (yaw_error=0, roll_error=rollE)   
                self.camera_decrease()
                ###### for zoom middle og the height...............................................
                if  abs(0.5 * cv_image.shape[0]- center[1]) < 20: # if the start sign lay in the middle of the window height or h/2
                    self.counter_zoom = self.counter_zoom +1 
                    # print('counter_zoom')
                    if self.counter_zoom > 4: #low pass filter
                        self.counter_zoom = 0                                           
                        self.twist.linear.x = - 0.009
                        self.twist.linear.y = 0.0
                        self.twist.angular.z = -0.1   
                        if self.autonomous_flag:
                            print('true')
                            for i in range(1,500): 
                                self.pub_vel.publish(self.twist)
                        self.movement_flag = 'line'
                        self.was_line = True
                        self.start_rope_toggle = False


            box = np.int0(cv2.boxPoints(blackbox))             
            cv2.drawContours(cv_image, [box], 0, (200, 200, 100), 3)
            # cv2.putText(cv_image, "yawE: " + str(00), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            # cv2.putText(cv_image, "rollE: " + str(rollE), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.line(cv_image, (center[0], center[1]), (center[0],center[1]+100), color=(255, 0, 0), thickness=3)
           
    # Detect the line and piloting
    def line_detect(self, cv_image):
        # Create a line mask
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)     
        line_mask = cv2.inRange(cv_image_hsv, self.lower_line, self.upper_line)       
        line_mask = cv2.dilate(line_mask, np.ones((5, 5), np.uint8), iterations=9)
        line_mask = cv2.erode(line_mask, np.ones((5, 5), np.uint8), iterations=5)       
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8), iterations=1)
         
        contours_blk, _ = cv2.findContours(line_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)       
        contours_blk.sort(key=cv2.minAreaRect)

        key =  cv2.waitKey(1)
        self.keyboard_function(key)        

        if len(contours_blk) > 0 :
            self.was_line = 1            
            blackbox = cv2.minAreaRect(contours_blk[-1])                        
            setpoint = cv_image.shape[1] / 2 # w/2
            
            center = (int(blackbox[0][0]),int(blackbox[0][1])) 
            width  = int(blackbox[1][0])
            height = int(blackbox[1][1])
            yawE  = int(blackbox[2])
            rollE = int(center[0] - setpoint)

            if abs(yawE)<=90: 
                if width > height:
                    yawE = yawE - 90                    
            else:                
                print('recovery dont yaw')         
                        
            self.controller (yaw_error=yawE, roll_error=rollE)
            
            box = np.int0(cv2.boxPoints(blackbox))             
            cv2.drawContours(cv_image, [box], 0, (200, 200, 100), 3)
           
            cv2.line(cv_image, (center[0], center[1]), (center[0],center[1]+100), color=(255, 0, 0), thickness=3)
      

    def stop_detect(self, cv_image):
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)     
        line_mask = cv2.inRange(cv_image_hsv, self.lower_line_stop, self.upper_line)       
        line_mask = cv2.dilate(line_mask, np.ones((5, 5), np.uint8), iterations=9)
        # line_mask = cv2.erode(line_mask, np.ones((5, 5), np.uint8), iterations=5)       
        # line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8), iterations=1)
         
        contours_blk, _ = cv2.findContours(line_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)       
        contours_blk.sort(key=cv2.minAreaRect)

        cv2.imshow("mask window", line_mask)
        key =  cv2.waitKey(1)
        self.keyboard_function(key)        

        if len(contours_blk) > 0 :
            self.was_line = 1            
            blackbox = cv2.minAreaRect(contours_blk[-1])   
            center = (int(blackbox[0][0]),int(blackbox[0][1])) 
            
            left_right_error = abs(int(center[0] - (cv_image.shape[1] / 2)))  #robot position - w/2
            up_down_error = (cv_image.shape[1] - int(center[1]))              #center in bottom
            
            if left_right_error < 100 and up_down_error > 100:
                print('finished')
   
    # Zoom-in the image
    def zoom(self, cv_image, scale):
        height, width, _ = cv_image.shape
        # print(width, 'x', height)
        # prepare the crop
        centerX, centerY = int(height / 2), int(width / 2)
        radiusX, radiusY = int(scale * height / 100), int(scale * width / 100)

        minX, maxX = centerX - radiusX, centerX + radiusX
        minY, maxY = centerY - radiusY, centerY + radiusY

        cv_image = cv_image[minX:maxX, minY:maxY]
        cv_image = cv2.resize(cv_image, (width, height))

        return cv_image

    # Image processing @ 10 FPS
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if self.movement_flag == 'start':  
            self.start_detect(cv_image)

        elif self.movement_flag == 'line':  
            cv_image = self.zoom(cv_image, scale=20) ####////////////////////////////////////
            self.line_detect(cv_image)
        
        # cv2.putText(cv_image, "pitch Vel: " + str(self.twist.linear.x ), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
        # cv2.putText(cv_image, "yaw   Vel: " + str(self.twist.angular.z ), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
        # cv2.putText(cv_image, "roll  Vel: " + str(self.twist.linear.y ), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)            
        # cv2.putText(cv_image, "movement flag: " + str(self.movement_flag), (10, 120), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
        # cv2.putText(cv_image, "battery: " + str(self.battery) + "%", (10, 160), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 0),thickness=2)
        cv2.imshow("Image window", cv_image)
        key =  cv2.waitKey(1)
        self.keyboard_function(key)             
                
def main():
    rospy.init_node('image_converter', anonymous=True)
    lf =  line_follower()
    lf.image_sub = rospy.Subscriber('/bebop/image_raw', Image, lf.callback)
    time.sleep(3)
    lf.cam_down()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown('interrupt')         
        
if __name__ == '__main__':
    main()