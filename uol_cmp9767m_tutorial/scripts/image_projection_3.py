#!/usr/bin/env python

# Python libs
import sys, time

# OpenCV
import cv2
from sklearn.cluster import DBSCAN

# Ros libraries
import roslib, rospy, image_geometry, tf
import numpy as np
# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import imutils
class image_projection:
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # (84.1/1920) / (70.0/512)
    color2depth_aspect = (84.1/1920) / (70.0/512)
    glo_array=[]

    def __init__(self):    
        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

        rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
            Image, self.image_color_callback)

        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)

        self.tf_listener = tf.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print (e)

        kernelOpen=np.ones((10,10))
        kernelClose=np.ones((20,20))

        image_colorHSV=cv2.cvtColor(image_color,cv2.COLOR_BGR2HSV)
        # detect a red blob in the color image
        image_mask = cv2.inRange(image_colorHSV, (100,30,55), (255,255,255))

        image_maskClose=cv2.morphologyEx(image_mask,cv2.MORPH_CLOSE,kernelClose)
        image_maskOpen=cv2.morphologyEx(image_maskClose,cv2.MORPH_OPEN,kernelOpen)

        conts = cv2.findContours(image_maskOpen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
        conts = imutils.grab_contours(conts)


        

        # calculate moments of the binary image
        # M = cv2.moments(image_mask)

        
        # cv2.drawContours(image_color,conts_rectangle,-1,(255,0,0),1)
        for i in range (len(conts)):
                x,y,w,h=cv2.boundingRect(conts[i])
                cv2.rectangle(image_color,(x,y),(x+w,y+h),(0,0,255), 2)# we draw a box around each contour 
                cv2.putText(image_color, str(i+1),(x,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0))#count the number of contours there are
                    
        # for c in conts:
        #     try:
        for c in conts:
            try:  
                M = cv2.moments(c)
                if M["m00"] == 0:
                    print ('No object detected.')
                    return
                # calculate the y,x centroid
                image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
                # "map" from color to depth image
                depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect,
                    image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect)
                # get the depth reading at the centroid location
                depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
                
                print ('image coords: ', image_coords)
                print ('depth coords: ', depth_coords)
                print ('depth value: ', depth_value)

                if np.isnan(depth_value):
                    continue
                # calculate object's 3d location in camera coords
                camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords
                camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth
                
                print ('camera coords: ', camera_coords)

                #define a point in camera coordinates
                object_location = PoseStamped()
                # object_location.header.frame_id = frame_id
                object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"
                object_location.pose.orientation.w = 1.0
                object_location.pose.position.x = camera_coords[0]
                object_location.pose.position.y = camera_coords[1]
                object_location.pose.position.z = camera_coords[2]


            # publish so we can see that in rviz
                self.object_location_pub.publish(object_location)


            # print out the coordinates in the map frame
                p_camera = self.tf_listener.transformPose('map', object_location)

                print ('map coords: ', p_camera.pose.position)
                print ('')
                #self.glo_array.append([round(p_camera.pose.position.x,2),round(p_camera.pose.position.y,2),round(p_camera.pose.position.z,2)])
                self.glo_array.append([p_camera.pose.position.x,p_camera.pose.position.y,p_camera.pose.position.z])
                #print(self.glo_array)
                #print('lenght', len(self.glo_array))

                # filter(lambda v: v==v, self.glo_array)
                epsilon = 0.05
                min_space = 20
                DB = DBSCAN(eps = epsilon, min_samples=min_space).fit(self.glo_array)
                No_clusters = len(np.unique(DB.labels_))
                print('number of cluster', No_clusters)

                # if self.visualisation:
                #     # draw circles
                #     cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
                #     cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

                #resize and adjust for visualisation
                #image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
                #image_depth *= 10.0/10.0 # scale for visualisation (max range 10.0 m)

                cv2.imshow("image depth", image_depth)
                cv2.imshow("image color", image_color)
                # cv2.imshow("image_mask", image_mask)

                # image_mask
                cv2.waitKey(1)
            except:
                continue

            # continue
        # calculate object's 3d location in camera coords
        # camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
        # camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
        # camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

        # print ('camera coords: '), camera_coords

        
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
