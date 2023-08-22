#!/usr/bin/env python


import cv2
import sys
import numpy as np
import time
import math
import cv2, cv_bridge
from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from pid import PID


class land():
    def __init__(self):
        
        self.lower_hsv = np.array([103, 178, 132]) 
        self.upper_hsv = np.array([137, 255, 255])

        self.Pyaw = 0.01
        self.Iyaw = 0.001
        self.Dyaw = 0
        self.bridge = cv_bridge.CvBridge()        
        self.twist = Twist()    
        self.fwd = Twist()   
              
        self.camera_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)   
        self.pub_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.land_pub = rospy.Publisher('bebop/land', Empty, queue_size=10) 
        self.emergency_pub = rospy.Publisher('bebop/reset', Empty, queue_size=10)      
        self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback, queue_size=10)        
        self.land_counter = 0
        time.sleep(1)

        self.Proll = 0.00025
        self.Iroll = 0.0004
        self.Droll = 0.0

        self.Pyaw = 0.01
        self.Iyaw = 0.001
        self.Dyaw = 0

        self.pid_roll = PID(self.Proll, self.Droll, self.Iroll, -0.1, 0.1, -0.1, 0.1) 
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.5, 0.5, -0.1, 0.1)  
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.04, 0.04, -0.1, 0.1)
        self.turn_toggle = True
        self.camera_angle = -50
        self.camera_down_start_time = time.time()
        self.go_to_H_toggle = False
        self.autonomous_flag = False
        self.contour_counter = 0
        self.counter_drift = 0
        self.altitude = 0
        self.decrease_start_time = time.time()
        self.decrease_time = 0

    def odom_callback(self, data):
        self.altitude = np.round(data.pose.pose.position.z, 3)
        
    
    def camera_decrease(self):
        if  time.time() -  self.camera_down_start_time > 1:
            self.camera_angle = self.camera_angle - 0.5
            if self.camera_angle < -65:
                self.camera_angle = -65
            self.cam_down(self.camera_angle)
            self.camera_down_start_time = time.time()

    def decrease(self):
        self.decrease_time = time.time()
        if self.decrease_time - self.decrease_start_time > 1:
            decrease_twsit = Twist()
            decrease_twsit.linear.z = -0.2 # decrease altitude 000000000000000000000000000000000000000000
            self.pub_vel.publish(decrease_twsit)
            self.decrease_start_time = time.time()


    def callback(self, data):                  
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        self.H_detect(cv_image)
        

        cv2.imshow("Adjustment", cv_image )         
        k = cv2.waitKey(1)  
        if k == ord('o'):           
            self.autonomous_flag = not self.autonomous_flag        
        if k == 27:  # close on ESC key
            cv2.destroyAllWindows()
            rospy.signal_shutdown('interrupt')  
        
    def H_detect(self, cv_image):        
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)     
        start_mask = cv2.inRange(cv_image_hsv, self.lower_hsv, self.upper_hsv)       
        start_mask = cv2.dilate(start_mask, np.ones((5, 5), np.uint8), iterations=9)
        start_mask = cv2.erode(start_mask, np.ones((5, 5), np.uint8), iterations=5)       
        start_mask = cv2.morphologyEx(start_mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8), iterations=1)
         
        contours_blk, _ = cv2.findContours(start_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)       
        contours_blk.sort(key=cv2.minAreaRect)

        if self.turn_toggle:
            turn_twist = Twist()
            turn_twist.angular.z =  -0.05  ###########
            if self.autonomous_flag:
                self.pub_vel.publish(turn_twist)

        if len(contours_blk) > 0 :           
            self.was_line = 1            
            blackbox = cv2.minAreaRect(contours_blk[-1])                        
            setpoint = cv_image.shape[1] / 2 # w/2
            self.height = cv_image.shape[0]
            center = (int(blackbox[0][0]),int(blackbox[0][1]))              
            rollE = int(center[0] - setpoint) 
            yawE  = int(blackbox[2]) - 90
            pitchE = cv_image.shape[0] - center[1] #h/2
            self.contour_height =  center[1]        
            self.contour_counter = self.contour_counter + 1
            # 0.5 w for turning left .................................
            if 0.5 * cv_image.shape[1] - center[0] < 20 and self.contour_counter > 10 and  self.go_to_H_toggle==False:      
                print('go to start toggle')          
                self.contour_counter = 0  
                self.go_to_H_toggle = True  
                self.turn_toggle = False           
            
            if self.go_to_H_toggle: 
                      
                self.controller (yaw_error=yawE, roll_error=rollE)   
                if rollE<100 : ############################################# for land ##########
                    self.land_counter = self.land_counter + 1
                    if self.land_counter >= 10:
                        self.land()
                self.camera_decrease()
                self.decrease()
               
                print(rollE)
               
            box = np.int0(cv2.boxPoints(blackbox))             
            # cv2.drawContours(cv_image, [box], 0, (200, 200, 100), 3)

    def land(self):
        msg = Empty()
        for i in range(1,5):
            self.land_pub.publish(msg)

    def controller(self, yaw_error, roll_error):

        self.pid_roll = PID(self.Proll, self.Droll, self.Iroll, -0.3, 0.3, -0.1, 0.1) 
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.15, 0.15, -0.1, 0.1) 
        
        yaw_treshold = 10
        roll_treshold = 75
        # if self.starter_drift_flag:
        #     self.twist.linear.x = 0.09
              
        
        if abs(roll_error) <= roll_treshold and abs(yaw_error)<= yaw_treshold:            
           
            self.twist.linear.x =  0.0017 #sasan            
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0  
        else:
            self.twist.linear.x =  0.017 #sasan                
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0         
            self.stateFlag = 'pitch'
                
        if abs(roll_error) > roll_treshold:
            # self.twist.linear.x = 0 #abs(self.pid_roll.update(roll_error)) * 0.9 for drift of roll ratio rate 
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
    
   

    def cam_down(self, angle):
        cam = Twist()
        cam.angular.y = angle 
        self.camera_pub.publish(cam)

if __name__ == '__main__':  
    rospy.init_node('takeoff_adjustment', anonymous=True)    
    
    la = land()   
    rospy.Subscriber('/bebop/image_raw', Image, la.callback)       
    time.sleep(3)            
         
    rospy.spin()
