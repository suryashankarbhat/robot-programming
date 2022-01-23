from mimetypes import init
# from queue import Empty
from unicodedata import name
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# from cv2 import namedWindow, resize, INTER_CUBIC, createTrackbar, getTrackbarPos, cvtColor, COLOR_BGR2HSV, inRange, bitwise_not, bitwise_and, imshow, waitKey, destroyAllWindows

class Mask_Out_Green:
    cv2.namedWindow('Trackbars')
    cv2.resizeWindow('Trackbars', 600, 300)
    cv2.createTrackbar("H_high", "Trackbars", 107, 255, lambda x:x)
    cv2.createTrackbar("H_low", "Trackbars", 100, 179, lambda x:x)
    cv2.createTrackbar("S_high", "Trackbars", 255, 255, lambda x:x)
    cv2.createTrackbar("S_low", "Trackbars", 18, 255, lambda x:x)
    cv2.createTrackbar("V_high", "Trackbars", 255, 255, lambda x:x)
    cv2.createTrackbar("V_low", "Trackbars", 46, 255, lambda x:x)


    def __init__(self):
        self.bridge = CvBridge()
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_front_camera/hd/image_color_rect", Image, self.callback)
    
    def callback(self, data):
        kernel = np.ones((5,5))

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        

        h_high = cv2.getTrackbarPos("H_high", 'Trackbars')
        h_low = cv2.getTrackbarPos("H_low", 'Trackbars')
        s_high = cv2.getTrackbarPos("S_high", 'Trackbars')
        s_low = cv2.getTrackbarPos("S_low", 'Trackbars')
        v_high = cv2.getTrackbarPos("V_high", 'Trackbars')
        v_low = cv2.getTrackbarPos("V_low", 'Trackbars')

        lower_color = np.array([h_low, s_low, v_low])
        higher_color = np.array([h_high, s_high, v_high])

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        color_selected = cv2.inRange(hsv_image,lower_color, higher_color)
        

        opening = cv2.morphologyEx(color_selected, cv2.MORPH_CLOSE,kernel)
        

        _,conts,_h = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(cv_image, conts, -1, (255,0,0),1)
        for i in range(len(conts)):
            x,y,w,h = cv2.boundingRect(conts[i])
            cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0,0,255), 2)
            cv2.putText(cv_image, str(i+1),(x,y+h), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))
        
        cv2.imshow('Trackbars', color_selected)
        cv2.imshow('HSV', hsv_image)
        cv2.imshow('Camera', cv_image)
        cv2.imshow('color_selected', opening)
        cv2.waitKey(30)
        print(len(conts))

if __name__ == '__main__':
    rospy.init_node('mask_out_object', anonymous=True)
    Mask_Out_Green()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
