# ros total classes test
#!/usr/bin/env pythonimport cv2
import sys
import numpy as np
import time
import math
import cv2, cv_bridge
import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from pid import PID


from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged, Ardrone3PilotingStateAltitudeChanged# For battery percentage
# sys.path.append("..")



class in_a_row():
    
    def __init__(self):
        
        ''' for global code'''
        self.twist = Twist()                       
        self.bridge = cv_bridge.CvBridge()                

        
        self.window_flag = False
        self.line_flag = False
        
        '''            init for global ROS             ''' 
        rospy.init_node('in_a_row', anonymous=True)        
        self.image_sub = rospy.Subscriber('/bebop/image_raw', Image, self.image_callback)  
        self.altitude_sub = rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged', 
                                             Ardrone3PilotingStateAltitudeChanged, self.altitude_callback, 
                                             queue_size=1, buff_size=2**28)        
      
        self.battery_sub = rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged',
                                            CommonCommonStateBatteryStateChanged, self.battery_callback)
        
        self.odom_sub = rospy.Subscriber('/bebop/odom', Odometry, self.odom_callback, queue_size=10)        
        self.camera_pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)           
        self.vel_pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
        
                       
       

        '''            init for take of ajustment             '''
        self.TA_angle_vector = list(np.ones(10))
        self.TA_norm_angle = 45
        self.TA_counter = 0      
        self.angle = []
        self.TA_lower_hsv = np.array([90, 160, 160]) 
        self.TA_upper_hsv = np.array([179, 255, 255]) 
          
        self.TA_Pyaw = 0.01
        self.TA_Iyaw = 0.001
        self.TA_Dyaw = 0
        
        self.TA_pid_yaw = PID(self.TA_Pyaw, self.TA_Dyaw, self.TA_Iyaw, -0.5, 0.5, -0.1, 0.1) 
        self.autonomous_flag = False
        
        self.adjustment_finished_flag = False


        '''            init for window             '''
        self.centerY = 0
        self.centerZ = 0
        self.window_error_y = 0
        self.window_error_z = 0
        self.window_y_vel = 0
        self.window_z_vel = 0
        self.battery = -1
        self.window_Pz = 0.006
        self.window_Iz = 0.005
        self.window_Dz = 0
        self.window_Py = 0.00038 #sasan
        self.window_Iy = 0.0 #sasan
        # self.window_Py = 0.00035 #mrl
        # self.window_Iy = 0.00025  #mrl
        self.window_Dy = 0.0
        self.window_Px = 0.0
        self.window_Ix = 0
        self.window_Dx = 0
        self.start_time = 0
        
        self.autonomous_flag = False
               
        self.window_vision_flag = True       
        self.window_update_flag = True   
        
        self.window_odom_y = 0
        self.window_odom_altitude = 0 
        self.window_mean_altitude = 0
        self.window_low_pass_alt = list(np.zeros(10))
        self.window_altitude = 1
        
        self.window_lower = np.array([92, 190, 80]) 
        self.window_upper = np.array([119, 255, 255])
                
        
        self.window_pid_y = PID(self.window_Py, self.window_Dy, self.window_Iy, -0.1, 0.1, -0.1, 0.1) 
        self.window_pid_z = PID(self.window_Pz, self.window_Dz, self.window_Iz, -0.3, 0.3, -0.1, 0.1) 
        
        self.window_finished_flag = False

        '''            init for line             '''



        '''            init for land             '''


    
    '''             for takeoff ajustment             '''
    '''             for takeoff ajustment             '''

    def TA_get_angle(self, x_orig, y_orig, x_des, y_des):
        deltaY = y_des - y_orig
        deltaX = x_des - x_orig   
        return math.atan2(deltaY, deltaX)*180/math.pi    
    
    def camera_control(self, angle):
        cam = Twist()
        cam.angular.y = angle
        self.camera_pub.publish(cam)
        time.sleep(3)
     

    def TA_lineDetect (self, cv_image):        
        h, w = cv_image.shape[:2]      
         
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)        
        mask = cv2.inRange( hsv , self.TA_lower_hsv , self.TA_upper_hsv)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=9)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=5)  
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)        
        contours,_ = cv2.findContours(mask, 1, 2) 
        
        if len(contours) != 0:
            biggest_contour = max(contours, key = cv2.contourArea)

            [vx,vy,x,y] = cv2.fitLine(biggest_contour, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((w-x)*vy/vx)+y)           
            cv2.line(cv_image,(w-1,righty),(0,lefty),(0,255,0),2)

            angle = self.TA_get_angle(0, lefty, w, righty)  
            self.TA_angle_vector.append(angle)
            self.TA_norm_angle = sum(self.TA_angle_vector)/len(self.TA_angle_vector)
            self.TA_angle_vector.pop(0)
            self.TA_norm_angle = round(self.TA_norm_angle, 3)

            if abs(self.TA_norm_angle) <= 0.5:
                self.TA_counter  = self.TA_counter  + 1
                print('counter=', self.TA_counter )
                if self.TA_counter  > 10:
                    self.adjustment_finished_flag = True
                    print(self.adjustment_finished_flag)

            self.twist.linear.x = 0
            self.twist.angular.z = -self.TA_pid_yaw.update(angle) 
            if self.autonomous_flag:
                self.vel_pub.publish(self.twist)

            
            print(self.TA_norm_angle)           
            print('angle = ', angle)
            print(lefty, righty)
            print('                         ')                                  
            cv2.putText(cv_image, text='angle : ' +str( self.TA_norm_angle), org=(10, 20), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Left   : ' +str( lefty), org=(10, 40), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='Right : ' +str( righty), org=(10, 60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.6, color=(0, 0, 0),thickness=2)
            cv2.putText(cv_image, text='flag  ' +str( self.autonomous_flag), org=(10, 80), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.65, color=(0, 0, 0),thickness=2)
            cv2.circle(cv_image, (w, righty), 10, (0,100,0), thickness=-1) 
            cv2.circle(cv_image, (0, lefty), 10, (0,100,0), thickness=-1) 
        else:
            self.TA_counter =0
            print('cant see the H')      
                    
    '''             for window             '''
    '''             for window             '''
    
   
    def window_detector(self, cv_image):
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        color_low = np.array(self.window_lower,dtype='uint8')       
        color_upper = np.array(self.window_upper,dtype='uint8')
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
                                
            else:                 
                x_init = int(x)
                y_init = int(y)
                x_final = int(x_init + w)
                y_final = int(y_init + h)                       
            
        else:
            x_init, y_init, x_final, y_final, center = 0,0,0,0,0
            x, y, w, h = 0,0,0,0
            print("cant see!!!!")

        center = [x+w//2, y+h//2]
        radius = 2       
        cv2.rectangle(output, (x_init, y_init), (x_final, y_final), (0, 255, 0), 2)
        cv2.circle(output, center, radius, (0, 250, 0), 2)
        cv2.circle(output, [self.centerY,self.centerZ], radius, (0, 0, 255), 10)
        # cv2.imshow("mask", output)
        # key =  cv2.waitKey(1)
        # self.keyboard_function(key)        
        cv2.rectangle(cv_image, (x_init, y_init), (x_final, y_final), (0, 255, 0), 2)
        cv2.circle(cv_image, center, radius, (0, 255, 0), 6)
        cv2.circle(cv_image, [self.centerY,self.centerZ], radius, (0, 0, 255), 10)
        
        cv2.putText(cv_image, "RollE: " + str(self.window_error_y), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "AltE: " + str(self.window_error_z), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "battery: " + str(self.battery) + "%", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "Yvel: " + str(self.window_y_vel), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        cv2.putText(cv_image, "Zvel: " + str(self.window_z_vel), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)

        cv2.putText(cv_image, "odomY: " + str(self.window_odom_y), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
        # cv2.putText(cv_image, "Zvel: " + str(self.window_z_vel), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 0),thickness=2)
       
        cv2.imshow('original',cv_image)    
        key =  cv2.waitKey(1)
        # self.keyboard_function(key)           

       
        print('X:',[self.window_Px, self.window_Ix, self.window_Dx])
        print('Y:',[self.window_Py, self.window_Iy, self.window_Dy])
        print('Z:',[self.window_Pz, self.window_Iz, self.window_Dz])
        print("autonomous flag:",self.autonomous_flag)
        print("update flag:",self.window_update_flag)
        print("vision flag:",self.window_vision_flag)
       
        print('goal altitude:', self.window_altitude) 
        print('real altitude:', self.window_mean_altitude) 
        print('goal y:', self.window_y) 
        print('real y:', self.window_odom_y)         
        print('                                                 ')
        

        return x_init, y_init, x_final, y_final, center

    def window_controller(self, goal, actual):      
      
        if self.window_vision_flag:
            self.window_error_y = goal[0] - actual[0] 
            self.window_error_z = goal[1] - actual[1]
            self.start_time = time.time()
        else:
            self.window_error_y = self.window_y  - self.window_odom_y 
            self.window_error_z = self.window_altitude - self.window_mean_altitude
                    
                    

        self.window_pid_y = PID(self.window_Py, self.window_Dy, self.window_Iy, -0.05, 0.05, -0.1, 0.1) 
        self.window_pid_z = PID(self.window_Pz, self.window_Dz, self.window_Iz, -0.3, 0.3, -0.1, 0.1) 
       
        self.window_y_vel = -np.round(self.window_pid_y.update(self.window_error_y)   ,4)
        self.window_z_vel = -(self.window_pid_z.update(self.window_error_z)   )
        
        if self.autonomous_flag == 1:
            twist = Twist()
            twist.linear.z = self.window_z_vel 
            twist.linear.y = self.window_y_vel 
            if abs(self.window_error_y) < (10):
                twist.linear.x = 0.02            
            elif abs(self.window_error_y) < (2):
                twist.linear.x = 0.04 
            elif abs(self.window_error_y) == 0:
                twist.linear.y = 0.0
            else:
                twist.linear.x = 0.0
            # twist.linear.x = 0.0
            self.vel_pub .publish(twist)           
                 
    def odom_callback(self, data):
        self.window_odom_y = np.round(data.pose.pose.position.y, 5)
        if (abs(self.window_error_y) < 1 and self.window_update_flag):
                self.window_y = self.window_odom_y      
  
    def altitude_callback(self, data):
        self.window_odom_altitude = round(data.altitude, 2)
        self.window_low_pass_alt.append(self.window_odom_altitude)
        self.window_low_pass_alt.pop(0)
       
        self.window_mean_altitude = np.mean(self.window_low_pass_alt)
        if (abs(self.window_error_z) < 1 and self.window_update_flag):
            self.window_altitude = self.window_mean_altitude            
   
    def battery_callback(self, data):
        self.battery = int(data.percent)

    def image_callback(self, data):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")                
                self.state_machine(cv_image)

                cv2.imshow("window", cv_image )                   
                key = cv2.waitKey(1)               
            except CvBridgeError as e:
                print(e)


    def state1(self, image):
        self.camera_control(-80)
        self.TA_lineDetect(image)
   
    def state2(self, image):
        self.camera_control(+3)
        self.window_detector(image)
        h, w = np.shape(image)[0:2]
        x1,y1, x2, y2, center_contour = self.window_detector(image)
        # flag setter for altitude update            
        if  (y2-y1) / h > 0.68:                                                             # if contour riches to x percent of the window:
            self.update_flag = False  
            self.vision_flag = False                                              # go with vision data in far distances                
                                                                                
        self.centerY = w//2
        self.centerZ = h//2            
        self.mean_altitude = np.mean(self.low_pass_alt)          
    
        self.window_controller(center_contour, [self.centerY,self.centerZ])
        if  (time.time() - self.start_time ) > 10:                
            self.window_finished_flag = True
            print(self.window_finished_flag)
    

    def state_machine(self, image):
        if not self.adjustment_finished_flag:            
            self.state1(image)
            

        # if self.adjustment_finished_flag == True and self.window_finished_flag == False:
        #     self.state2(image)
           
                


def main():
    try:     
        IAR = in_a_row()

        rospy.spin()         
    except (KeyboardInterrupt, EOFError):
        cv2.destroyAllWindows()
        rospy.signal_shutdown('keyboard')
        sys.exit()        


if __name__ == '__main__':   
   main()