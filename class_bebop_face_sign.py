import cv2 as cv
from ultralytics import YOLO
import numpy
import random
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class face_sign_detection():
          
    def __init__(self) -> None:

        self.path = '/home/arshix/bebop_ws/'
        self.model=YOLO(self.path+'src/open2023/scripts/bebop_in_a_row/best.pt','v8')
        self.bridge = CvBridge()
        
        
        myfile = open(self.path+'/src/open2023/scripts/bebop_in_a_row/classes1.txt','r')
        data = myfile.read()
        self.class_list = data.split("\n")
        # print(self.class_list)
        myfile.close()
     

    # def reader(self):
        # myfile = open('classes1.txt','r')
        # self.data = myfile.read()
        # class_list = self.data.split("\n")
        # myfile.close()
        # return class_list

    def detection(self, cv_img):
        pass

    def face_sign_callback(self, ros_img):
        cv_img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        
        detection_colors = []
        for i in range(len(self.class_list)):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            detection_colors.append((B,G,R))

        detect_params = self.model.predict(source=[cv_img] , conf = 0.45 , save=False) #source="inference/images/frame.png"
        DP = detect_params[0].cpu().numpy()       
        
        if len (DP) != 0 :
            for i in range(len(detect_params[0])):                   
                # print(len(detect_params[0]))
                boxes = detect_params[0].boxes
                box = boxes[i]
                clsID = box.cls.cpu().numpy()[0]
                conf = box.conf.cpu().numpy()[0]
                bb = box.xyxy.cpu().numpy()[0]
                cv.rectangle(
                    cv_img,
                    (int(bb[0]), int(bb[1])),
                    (int(bb[2]), int(bb[3])),
                    detection_colors[int(clsID)],
                    3)

                font = cv.FONT_HERSHEY_COMPLEX
                rnd = random.randint(1,10000)                
                cv.putText(
                    cv_img,
                    self.class_list[int(clsID)] + " " + str(round(conf, 3)) + "%",
                    (int(bb[0]), int(bb[1]) - 10),
                    font,
                    1,
                    (255, 255, 255),
                    2)
                cv.imwrite(self.path+'src/open2023/scripts/bebop_in_a_row/detected/'+ self.class_list[int(clsID)] + str(rnd)+'.jpg', cv_img)
        cv.imshow('ObjectDetection',cv_img)
        if cv.waitKey(1) == 27 :                
            exit()
            

            

if __name__ == '__main__':
    try:
        fsd = face_sign_detection()
        rospy.init_node('bebop_face_sign', anonymous=True)
        rospy.Subscriber('/bebop/image_raw', Image, fsd.face_sign_callback)
        time.sleep(1)

        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv.destroyAllWindows()
        rospy.signal_shutdown('interrupt')         
  
