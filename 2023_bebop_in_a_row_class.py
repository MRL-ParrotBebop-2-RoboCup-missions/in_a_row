#!/usr/bin/env python

import time
import cv2, cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from class_bebop_takeoff_adjustment import TakeOffAdjustment
from class_bebop_window import bebop_window
from class_bebop_face_sign import face_sign_detection
from class_bebop_line_follower_contour import line_follower
from class_bebop_land import land

# from cv_bridge import CvBridge, CvBridgeError

class bebop_state_machine():
    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)     
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.callback)  
        self.camera_pub = rospy.Publisher('bebop/camera_control', Twist, queue_size=10)  
        self.land_pub = rospy.Publisher('bebop/land', Empty, queue_size=10)          
        self.emergency_pub = rospy.Publisher('bebop/reset', Empty, queue_size=10)    
        self.face_sign = face_sign_detection()               
        time.sleep(2)
        self.bridge = cv_bridge.CvBridge()        
        self.autonomous_flag = False
        self.first_mission_flag = False
        self.second_mission_flag = False
        self.third_mission_flag = False
        self.adjustment_finished_flag = False
        self.toggle1 = True  
        self.toggle2 = True 
        self.toggle3 = True
        self.toggle4 = True
        self.time_toggle = True 
        print('initiating')    
        self.start_time = time.time()      
        self.sleep_state = True    
        self.start_sleep = 0
        self.fps_counter = 0
        self.minus_seventy_five_counter = 0
        self.start_time_camera = time.time()


    # def emergency(self):
    #     emergency = Empty()
    #     self.emergency_pub.publish(emergency)            
    #     time.sleep(0.1)
    #     exit()      
    def camera_publisher(self, angle):
        cam = Twist()
        cam.angular.y = angle  
        for i in range(1, 5):              
            self.camera_pub.publish(cam)              
            # print('camera heading to', angle)            


    def state1(self, img):
        
        if self.toggle1:                   
            print('state 1 is starting')   
            self.camera_publisher(-70)   
                          
            self.TA = TakeOffAdjustment()                            
            self.toggle1 = False   
        self.adjustment_finished_flag = self.TA.adjustment_finished_flag
        self.first_mission_flag = self.TA.forward_finished_flag        
        
        if self.adjustment_finished_flag and self.first_mission_flag == False:      
            self.camera_publisher(+3)

        self.autonomous_flag = self.TA.autonomous_flag        
        self.TA.callback(img)
        
        if self.first_mission_flag:
           
            print('state 1 is finished')
            cv2.destroyAllWindows()
            del self.TA          
                     

    def state2(self, img):
        self.my_sleep(4)
        # print('bi namus', self.autonomous_flag)
        if self.toggle2:                     
            self.WINDOW = bebop_window()                                 
            print('state 2 is started ....')         
            self.toggle2 = False
            print('toggle2',  self.toggle2)
            
        
        self.WINDOW.autonomous_flag = self.autonomous_flag     
        self.second_mission_flag = self.WINDOW.window_finished_flag 
        if not self.WINDOW.vision_flag:
            self.camera_publisher(-50)            
        
        self.WINDOW.callback(img)
        if self.second_mission_flag:
            print('state 2 finished')
            cv2.destroyAllWindows()                
            del self.WINDOW        
       
    def state3(self, img):      
                
        if self.toggle3:                     
            self.Line = line_follower()                                 
            print('state 3 is started ....')         
            self.toggle3 = False
            print('toggle3',  self.toggle3)
            self.start_time_camera = time.time()
        # self.autonomous_flag = self.Line.autonomous_flag     
        # for single test
        self.Line.autonomous_flag = self.autonomous_flag     
        # self.third_mission_flag= self.Line.line_finished_flag
        self.Line.callback(img)

        if   time.time() - self.start_time_camera > 60:      
                self.camera_publisher(-75)      

        self.Line.callback(img)
        
        # if self.third_mission_flag:           
        if   time.time() - self.start_time_camera > 300:  
            print('state 3 finished')
            self.third_mission_flag = True
            cv2.destroyAllWindows()     

    def detection(self, image):
        self.fps_counter = self.fps_counter + 1
        if self.fps_counter / 30 == 1:
            self.start_time = time.time()
            self.face_sign.face_sign_callback(image)
            self.fps_counter = 0


    def state4(self, img):
        if self.toggle4:                   
            print('state 4 is starting')   
            self.camera_publisher(-50)                             
            self.land = land()                            
            self.toggle4 = False
           
            self.land.autonomous_flag = self.autonomous_flag
        # self.autonomous_flag = self.land.autonomous_flag 
        self.land.autonomous_flag = self.autonomous_flag       
        self.land.callback(img)

    def callback(self, img):  
      
        # self.state3(img)
        self.detection(img)
        if  self.first_mission_flag == False:
            self.state1(img)            

        else:      
            if self.second_mission_flag == False:
                self.state2(img)
            
            else:
                if self.third_mission_flag == False:
                        self.state3(img)
                else:
                    self.state4(img)
                              

    def my_sleep(self, duration):
        if self.sleep_state:
            if self.time_toggle:
                self.start_sleep = time.time()
                self.time_toggle = False
                self.autonomous_flag = False   
            print('sleep')
            if duration <=  time.time() - self.start_sleep:
                # print('wakeup                        ==')
                self.autonomous_flag = True
                self.time_toggle = True
                self.sleep_state = False
        # print('void')


if __name__ == '__main__':    
   
    SM = bebop_state_machine() 
    
    rospy.spin()

