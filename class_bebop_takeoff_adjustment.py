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

class TakeOffAdjustment :
    
    def __init__(self) :
        self.angle_vector = list(np.ones(10))
        self.norm_angle = 45
        self.counter = 0    
  
        self.error = []
        self.angle = []
        self.lower_hsv = np.array([105, 164, 116]) 
        self.upper_hsv = np.array([123, 255, 255]) 
          
        self.Pyaw = 0.01
        self.Iyaw = 0.001
        self.Dyaw = 0

        self.bridge = cv_bridge.CvBridge()        
        self.twist = Twist()    
        self.fwd = Twist()   
        
        self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback, queue_size=10)        
        # self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.callback)  
        self.camera_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)   
        self.pub_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size=10)   
        time.sleep(1)
        self.pid_yaw = PID(self.Pyaw, self.Dyaw, self.Iyaw, -0.04, 0.04, -0.1, 0.1) 
        
        self.counter_toggle = True
        self.autonomous_flag = False        
        self.adjustment_finished_flag = False
        self.forward_finished_flag = False        
        self.qeue_toggle = True

        # self.time_toggle = True 
        # self.start_time = time.time()      
        self.sleep_state = True    
        # self.start_sleep = 0

    # def my_sleep(self, duration):
    #     if self.sleep_state:
    #         if self.time_toggle:
    #             self.start_sleep = time.time()
    #             self.time_toggle = False
    #             self.autonomous_flag = False   
    #         print('sleep')
    #         if duration <=  time.time() - self.start_sleep:
                
    #             self.autonomous_flag = True
    #             self.time_toggle = True
    #             self.sleep_state = False
    #             print('wakeup                        ==')
                

    def take_off(self):
        if self.autonomous_flag:
            takeoff = Empty()
            self.takeoff_pub.publish(takeoff)                        
            print("drone is taked off!!") 
            time.sleep(1)      
            rospy.sleep(1)
   
    def get_angle(self, x_orig, y_orig, x_des, y_des):
        deltaY = y_des - y_orig
        deltaX = x_des - x_orig   
        return math.atan2(deltaY, deltaX)*180/math.pi    
    
    def callback(self, data):                  
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        self.lineDetect(cv_image)

        cv2.imshow("Adjustment", cv_image )         
        k = cv2.waitKey(1)  
        if k == ord('o'):           
            self.autonomous_flag = not self.autonomous_flag
        if k == ord('t'):                
                self.take_off()
        if k == 27:  # close on ESC key
            cv2.destroyAllWindows()
            rospy.signal_shutdown('interrupt')  
       
    def lineDetect (self, cv_image):        
       
        h, w = cv_image.shape[:2]   
       
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)        
        mask = cv2.inRange( hsv , self.lower_hsv , self.upper_hsv)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=9)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=5)  
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)        
        contours,_ = cv2.findContours(mask, 1, 2) 
        self.action()
        if len(contours) != 0:
            biggest_contour = max(contours, key = cv2.contourArea)

            [vx,vy,x,y] = cv2.fitLine(biggest_contour, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((w-x)*vy/vx)+y)           
            cv2.line(cv_image,(w-1,righty),(0,lefty),(0,255,0),2) #line commnented*****

            angle = self.get_angle(0, lefty, w, righty)  + 0   #rightward yaw offset drift
            self.angle_vector.append(angle)
            self.norm_angle = sum(self.angle_vector)/len(self.angle_vector)
            self.angle_vector.pop(0)
            self.norm_angle = round(self.norm_angle, 3)

            if   abs(self.norm_angle)  < 0.5 and self.counter_toggle:         
                self.counter  = self.counter  + 1
                print('counter=', self.counter )
                if self.counter  > 40 and self.counter_toggle:                       
                    self.counter_toggle = False
                    self.adjustment_finished_flag = True 
                    self.start_to_odom = self.odom_x
                   
            
            cv2.putText(cv_image, text='angle : ' +str( self.norm_angle), org=(10, 20), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Left   : ' +str( lefty), org=(10, 40), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Right : ' +str( righty), org=(10, 60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='flag  ' +str( self.autonomous_flag), org=(10, 80), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.65, color=(0, 0, 0),thickness=2)
            # cv2.circle(cv_image, (w, righty), 10, (0,100,0), thickness=-1) 
            # cv2.circle(cv_image, (0, lefty), 10, (0,100,0), thickness=-1) 
        else:
            self.counter =0
            # print('cant see the H')
       
    def action(self):    
        #------------------------------------------------------forwarding action ------------------------------------------------#                       
        if self.autonomous_flag:

            if self.adjustment_finished_flag:            #adjusting finished and go forward for x seconds
                print(self.odom_x - self.start_to_odom )
                
                # 
                if  abs(self.odom_x - self.start_to_odom) > 0.4:    # 0.7 odometry metric difference                         
                                                #forwarding finished after x seconds and stop right there
                    if self.qeue_toggle:                      
                        
                        self.fwd.linear.x = -0.025 
                        self.fwd.linear.y = 0.0       

                        for i in range(1, 5):    
                            self.pub_vel.publish(self.fwd)
                        self.qeue_toggle = False                       
                    self.forward_finished_flag = True  
                    # print('forward_finished_flag = ', self.forward_finished_flag)  
                else:
                    self.fwd.linear.x = 0.03     ##sasan                                             
                    self.pub_vel.publish(self.fwd)       
                 
            #---------------------------------------------------adjusting action---------------------------------------------------#        
            else:                    
                self.twist.linear.x = 0
                self.twist.angular.z = -self.pid_yaw.update(self.norm_angle) 
                self.pub_vel.publish(self.twist)

    def odom_callback(self, data):
        self.odom_x = np.round(data.pose.pose.position.x, 3)
        # print(self.odom_x)
        
               

    def cam_down(self):
        cam = Twist()
        cam.angular.y = -70 
        self.camera_pub.publish(cam)

        

if __name__ == '__main__':  
    rospy.init_node('takeoff_adjustment', anonymous=True)    
    
    TA = TakeOffAdjustment()   
    rospy.Subscriber('/bebop/image_raw', Image, TA.callback)       
    time.sleep(1)       
    TA.cam_down()
    time.sleep(3)            
         
    rospy.spin()


