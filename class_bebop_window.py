#!/usr/bin/env python
import sys
import rospy
import cv2

import time
import numpy as np
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Float32  # For error/angle plot publishing
from open2023.msg import bebop_global_logger
from sensor_msgs.msg import Image
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged, Ardrone3PilotingStateAltitudeChanged# For battery percentage
from cv_bridge import CvBridge, CvBridgeError
# import json
import os
from pid import PID

# Frame rate (time to sleep)
       
class bebop_window:
    def __init__(self):
          
        self.centerY = 0
        self.centerZ = 0
        self.error_y = 0
        self.error_z = 0
        self.y_vel = 0
        self.z_vel = 0
        self.battery = -1
        self.Pz = 0.006
        self.Iz = 0.005
        self.Dz = 0
        self.Py = 0.0004 #sasan
        self.Iy = 0.0 #sasan
        self.Py = 0.00035 #mrl
        # self.Iy = 0.00025  #mrl
        self.Dy = 0.0
        self.Px = 0.0
        self.Ix = 0
        self.Dx = 0
        self.start_time = 0
        self.window_y = 0
        self.tune_flag = None
        self.autonomous_flag = False
               
        self.vision_flag = True       
        self.update_flag = True   

        self.bridge = CvBridge()
        self.odom_y = 0
        self.odom_altitude = 0 
        self.mean_altitude = 0
        self.low_pass_alt = list(np.zeros(10))
        self.window_altitude = 1
        
        self.lower = np.array([93, 153, 34]) 
        self.upper = np.array([124, 200, 155])

        
        self.vel_pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)             
        
       
        self.battery_sub = rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged',
                                            CommonCommonStateBatteryStateChanged, self.battery_callback)
        
        self.altitude_sub = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', 
                                             Ardrone3PilotingStateAltitudeChanged, self.altitude_callback, 
                                             queue_size=1, buff_size=2**28)        
               
        self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback, queue_size=10)        
        self.pid_y = PID(self.Py, self.Dy, self.Iy, -0.1, 0.1, -0.1, 0.1) 
        self.pid_z = PID(self.Pz, self.Dz, self.Iz, -0.3, 0.3, -0.1, 0.1) 
        self.pub_roll_error = rospy.Publisher('/bebop/roll_error', Float32, queue_size=1)

        self.window_finished_flag = False
    
    def keyboard_function(self, key):
        if  key == 27:  # close on ESC key
                cv2.destroyAllWindows()
                rospy.signal_shutdown('interrupt')   
                exit()  

        if key == ord('x'):
            self.tune_flag = 'x'        
        elif key == ord('y'):
            self.tune_flag = 'y'        
        elif key == ord('z'):
            self.tune_flag = 'z'        

        if key == ord('o'):
            self.autonomous_flag = not self.autonomous_flag    
    
        if self.tune_flag == 'z':
            if key == ord('q')  :
                self.Pz = round(self.Pz + 0.001, 4)
            elif key == ord('a')  :
                self.Pz = round(self.Pz - 0.001, 4)
                
            if key == ord('w')  :
                self.Iz = round(self.Iz + 0.001, 4)
            elif key == ord('s')  :
                self.Iz = round(self.Iz - 0.001, 4)
                
            if key == ord('e')  :
                self.Dz = round(self.Dz + 0.0001, 4)
            elif key == ord('d')  :
                self.Dz = round(self.Dz - 0.0001, 4)
            elif key == ord(' ')  :
                self.Pz, self.Iz, self.Dz = 0, 0, 0
            

        elif self.tune_flag == 'y':
            if key == ord('q') :
                self.Py = round(self.Py + 0.0001, 4)
            elif key == ord('a') :
                self.Py = round(self.Py - 0.0001, 4)            

            if key == ord('w') :
                self.Iy = round(self.Iy + 0.0001, 4)
            elif key == ord('s') :
                self.Iy = round(self.Iy - 0.0001, 4)
            
            if key == ord('e') :
                self.Dy = round(self.Dy + 0.00001, 6)
            elif key == ord('d') :
                self.Dy = round(self.Dy - 0.00001, 6)            
            if key == ord(' ') :
                self.Py, self.Iy, self.Dy = 0, 0, 0
        
        elif self.tune_flag == 'x':
            if key == ord('q') :
                self.Px = round(self.Px + 0.001, 4)
            elif key == ord('a') :
                self.Px = round(self.Px - 0.001, 4)            

            if key == ord('w') :
                self.Ix = round(self.Ix + 0.001, 4)
            elif key == ord('s') :
                self.Ix = round(self.Ix - 0.001, 4)
            
            if key == ord('e') :
                self.Dx = round(self.Dx + 0.001, 4)
            elif key == ord('d') :
                self.Dx = round(self.Dx - 0.001, 4)            
            if key == ord(' ') :
                self.Px, self.Ix, self.Dx = 0, 0, 0


    def cam_up(self):
        cam = Twist()
        cam.angular.y = 3.0
        self.pub_camup.publish(cam)    
   

    def window_detector(self, cv_image):
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        color_low = np.array(self.lower,dtype='uint8')       
        color_upper = np.array(self.upper,dtype='uint8')
        mask = cv2.inRange( hsv , color_low , color_upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=9)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=5)  
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        output = cv2.bitwise_and(cv_image , cv_image ,mask = mask)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            if ((w/h)>1) & ((w/h)<2) :
         
                x_init = int(x + (0.12 * w))
                y_init = int(y + (0.15 * h))
                x_final = int(x_init + (w*0.75)) #0.75
                y_final = int(y_init + (h*0.7))
                # print("w=", w, " h=",)
                
            else:                 
                x_init = int(x)
                y_init = int(y)
                x_final = int(x_init + w)
                y_final = int(y_init + h)                       
            
        else:
            x_init, y_init, x_final, y_final, center = 0,0,0,0,0
            x, y, w, h = 0,0,0,0
            # print("cant see!!!!")

        center = [x+w//2, y+h//2]
        radius = 2
        cv2.rectangle(output, (x_init, y_init), (x_final, y_final), (0, 255, 0), 2)
        cv2.circle(output, center, radius, (0, 250, 0), 2)
        cv2.circle(output, [self.centerY,self.centerZ], radius, (0, 0, 255), 10)
        #cv2.imshow("mask", output)
        key =  cv2.waitKey(1)
        self.keyboard_function(key)        
        cv2.rectangle(cv_image, (x_init, y_init), (x_final, y_final), (0, 255, 0), 2)
        cv2.circle(cv_image, center, radius, (0, 255, 0), 6)
        cv2.circle(cv_image, [self.centerY,self.centerZ], radius, (0, 0, 255), 10)
        self.pub_roll_error.publish(self.error_y)
        cv2.putText(cv_image, "RollE: " + str(self.error_y), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "AltE: " + str(self.error_z), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "battery: " + str(self.battery) + "%", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "Yvel: " + str(self.y_vel), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "Zvel: " + str(self.z_vel), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "odomY: " + str(self.odom_y), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        # cv2.putText(cv_image, "Zvel: " + str(self.z_vel), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
       
        cv2.imshow('original',cv_image)    
        key =  cv2.waitKey(1)
        self.keyboard_function(key)           
       
        return x_init, y_init, x_final, y_final, center

    def controller(self, goal, actual):      
      
        if self.vision_flag:
            self.error_y = goal[0] - actual[0] 
            self.error_z = goal[1] - actual[1]
            self.start_time = time.time()
        else:
            self.error_y = self.window_y  - self.odom_y 
            self.error_z = self.window_altitude - self.mean_altitude
                    
                    

        self.pid_y = PID(self.Py, self.Dy, self.Iy, -0.05, 0.05, -0.1, 0.1) 
        self.pid_z = PID(self.Pz, self.Dz, self.Iz, -0.3, 0.3, -0.1, 0.1) 
             
        # self.y_vel = -np.round(self.pid_y.update(self.error_y)   ,4)
        self.z_vel = -(self.pid_z.update(self.error_z)   )
        
        if self.autonomous_flag:            
            twist = Twist()
            twist.linear.z = self.z_vel 
            if self.error_y < 0:
                sign = 1            #move to left
            else:
                sign = -1           #move to right        

            # ---------------------------------------------------mrl---------------------------------------#
            if abs(self.error_y) >= 100:
                twist.linear.x = -0.001            
                twist.linear.y = sign * 0.020

            elif abs(self.error_y) < 100 and abs(self.error_y) >= 30: 
                twist.linear.x = -0.0005            
                twist.linear.y = sign * 0.015   

            elif abs(self.error_y) < 30 and abs(self.error_y) >= 10: 
                twist.linear.x = 0.01  
                twist.linear.y = sign * 0.0019

            elif abs(self.error_y) < 10 and abs(self.error_y) >= 1: 
                twist.linear.x = 0.013           
                twist.linear.y = sign * 0.0008

            elif not self.vision_flag: 
                twist.linear.x = 0.04          
                twist.linear.y = 0

            # #---------------------------------------------------sasan---------------------------------------#
            # if abs(self.error_y) >= 100:
            #     twist.linear.x = -0.001            
            #     twist.linear.y = sign * 0.025

            # elif abs(self.error_y) < 100 and abs(self.error_y) >= 30: 
            #     twist.linear.x = -0.001            
            #     twist.linear.y = sign * 0.017

            # elif abs(self.error_y) < 30 and abs(self.error_y) >= 10: 
            #     twist.linear.x = 0.011  
            #     twist.linear.y = sign * 0.0015

            # elif abs(self.error_y) < 10 and abs(self.error_y) >= 1: 
            #     twist.linear.x = 0.014           
            #     twist.linear.y = sign * 0.00075

            # elif not self.vision_flag: 
            #     twist.linear.x = 0.03          
            #     twist.linear.y = 0


            else:
                print('non else')          
            self.vel_pub .publish(twist)   
                       
                 
    def odom_callback(self, data):
        self.odom_y = np.round(data.pose.pose.position.y, 5)
        if (abs(self.error_y) < 2 and self.update_flag):
                self.window_y = self.odom_y      
               

    def altitude_callback(self, data):
        self.odom_altitude = round(data.altitude, 2)
        self.low_pass_alt.append(self.odom_altitude)
        self.low_pass_alt.pop(0)
       
        self.mean_altitude = np.mean(self.low_pass_alt)
        if (abs(self.error_z) < 1 and self.update_flag):
            self.window_altitude = self.mean_altitude            
              

    def battery_callback(self, data):
        self.battery = int(data.percent)
       

    def callback(self, img):
    
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        h, w = np.shape(cv_image)[0:2]
        x1,y1, x2, y2, center_contour = self.window_detector(cv_image)
        # flag setter for altitude update
        
        if  (y2-y1) / h > 0.68:   ##                                                          # if contour riches to x percent of the window:
            self.update_flag = False  
            self.vision_flag = False                                             # go with vision data in far distances                
                                                                                

        self.centerY = w//2
        self.centerZ = h//2            
        self.mean_altitude = np.mean(self.low_pass_alt)
        
    
        self.controller(center_contour, [self.centerY,self.centerZ])
        if  (time.time() - self.start_time ) > 6: #####depends on forward speed
            # print('window finished')
            stop_twist = Twist()
            self.vel_pub.publish(stop_twist)
            self.window_finished_flag = True
     
              
def main():    
    try:
        rospy.init_node('bebop_window', anonymous=True)
        
        window = bebop_window()
        window.image_sub = rospy.Subscriber('/bebop/image_raw', Image, window.callback)
        window.pub_camup = rospy.Publisher('bebop/camera_control', Twist, queue_size=10)
        time.sleep(1)
        window.cam_up()
       
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown('interrupt')         
            

if __name__ == '__main__':
     main()
