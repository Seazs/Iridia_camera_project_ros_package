#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import os
from ultralytics import YOLO
from ultralytics.engine.results import Results
from time import sleep
from math import tan

CAMERA_RATIO = 12/9 # 2592/1944 pixels -> 640/480 pixels
HEIGHT_PX = 480
WIDTH_PX = 640
FOV = 130 * 3.14159265358979323846 / 180 # radians
ROOM_HEIGHT = 2 # suppose the door is 2 meters high
ALPHA = 2 # exponent of the model

rospack = rospkg.RosPack()
package_path = rospack.get_path("up_pointing_camera")
weight_path = os.path.join(package_path, "src", "door_detection_weights.pt")

def get_center_of_object(obj):
    x1, y1, x2, y2 = obj.xyxy[0]
    return (x1 + x2) / 2, (y1 + y2) / 2


def get_real_position_from_detection_cubic(detection):
    visible_ceiling_length = tan(FOV/2)*ROOM_HEIGHT
    image_center_x, image_center_y = (WIDTH_PX/2, HEIGHT_PX/2)
    door_center_x, door_center_y = get_center_of_object(detection)
    #for cubic model
    dx = (door_center_x - image_center_x)
    dy = (door_center_y - image_center_y)

    if dx > 0:   
        x_distance = ROOM_HEIGHT*tan(FOV/2)*(dx/(WIDTH_PX/2))**(ALPHA)
    else:
        x_distance = -ROOM_HEIGHT*tan(FOV/2)*((-dx)/(WIDTH_PX/2))**(ALPHA)

    if dy > 0:
        y_distance = ROOM_HEIGHT*tan(FOV/2)*(dy/(WIDTH_PX/2))**(ALPHA)
    else:
        y_distance = -ROOM_HEIGHT*tan(FOV/2)*((-dy)/(WIDTH_PX/2))**(ALPHA)    
    return x_distance, y_distance





class CameraListener:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=10)
        self.door_detection_pub = rospy.Publisher("door_detection", Int16MultiArray, queue_size=1)
        self.bridge = CvBridge()
        self.image = None
        
        
        self.yolo = YOLO(weight_path)
        self.classes = self.yolo.names #list of classes

        self.confirmation_pub = rospy.Publisher("image_proccesed_confirmation", Empty, queue_size=1)
        sleep(1)
        self.confirmation_pub.publish(Empty())
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #apply the model
        result: Results = self.yolo.predict(cv_image, show= False)[0]
        for obj in result.boxes: #cls is the class of the object
            detected_class = self.classes[int (obj.cls)]
            rospy.loginfo(detected_class)
            if detected_class == "doors":
                
                # get the real position of the door
                x_distance, y_distance = get_real_position_from_detection_cubic(obj)
                

                # rospy.loginfo("center_x: {}, center_y: {}".format(center_x, center_y))
                rospy.loginfo("x_distance: {}, y_distance: {}".format(x_distance, y_distance))

                data_analysed = Int16MultiArray(data=[int(x_distance*100), int(y_distance*100)])
                self.door_detection_pub.publish(data_analysed)
            
        self.confirmation_pub.publish(Empty()) 

        

if __name__ == '__main__':
    rospy.loginfo("starting camera listener node")
    rospy.init_node('camera_listener')
    rospy.loginfo("Camera Listener Node has been started")
    camera_listener = CameraListener()
    rospy.spin()            