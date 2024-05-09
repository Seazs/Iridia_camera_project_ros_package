#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
from time import sleep
from math import tan

CAMERA_RATIO = 12/9 # 2592/1944 pixels -> 640/480 pixels
HEIGHT_PX = 480
WIDTH_PX = 640
# FOVX = 72 # degrees (withoout fisheye)
# FOVY = 54 # = 72 * 9/12 degrees(without fisheye)
FOV = 130 * 3.14159265358979323846 / 180 # radians
ROOM_HEIGHT = 2 # suppose the door is 2 meters high
ALPHA = 2 # exponent of the model

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
        self.yolo = YOLO("/home/alex/catkin_ws/src/up_pointing_camera/src/best.pt")
        self.classes = self.yolo.names #list of classes

        self.confirmation_pub = rospy.Publisher("image_proccesed_confirmation", Empty, queue_size=1)
        sleep(1)
        self.confirmation_pub.publish(Empty())
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #apply the model
        result: Results = self.yolo.predict(cv_image, show= True)[0]
        for obj in result.boxes: #cls is the class of the object
            detected_class = self.classes[int (obj.cls)]
            print(detected_class)
            rospy.loginfo(detected_class)
            if detected_class == "doors":
                #publish the detection

                # get the coordinates of the bounding box
                x1, y1, x2, y2 = obj.xyxy[0]
                # get the center of the door
                center_x, center_y = get_center_of_object(obj)

                # get the real position of the door
                x_distance, y_distance = get_real_position_from_detection_cubic(obj)
                

                rospy.loginfo("center_x: {}, center_y: {}".format(center_x, center_y))
                rospy.loginfo("x_distance: {}, y_distance: {}".format(x_distance, y_distance))
                # #draw a green circle around at the center of the door
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                cv2.circle(cv_image, (int(center_x), int(center_y)), 10, (0, 255, 0), -1)
                #write the class name on the image
                #cv2.putText(cv_image, detected_class, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("image", cv_image)

                data_analysed = Int16MultiArray(data=[1, 0, 0, 0])
                self.door_detection_pub.publish(data_analysed)
            
        
        self.confirmation_pub.publish(Empty()) 

        

if __name__ == '__main__':
    rospy.loginfo("starting camera listener node")
    rospy.init_node('camera_listener')
    rospy.loginfo("Camera Listener Node has been started")
    camera_listener = CameraListener()
    rospy.spin()

            

            
            


    # ca c'est qu'il est possible de faire avec les results

    # Attributes:
    #     orig_img (numpy.ndarray): Original image as a numpy array.
    #     orig_shape (tuple): Original image shape in (height, width) format.
    #     boxes (Boxes, optional): Object containing detection bounding boxes.
    #     masks (Masks, optional): Object containing detection masks.
    #     probs (Probs, optional): Object containing class probabilities for classification tasks.
    #     keypoints (Keypoints, optional): Object containing detected keypoints for each object.
    #     speed (dict): Dictionary of preprocess, inference, and postprocess speeds (ms/image).
    #     names (dict): Dictionary of class names.
    #     path (str): Path to the image file.

    # Methods:
    #     update(boxes=None, masks=None, probs=None, obb=None): Updates object attributes with new detection results.
    #     cpu(): Returns a copy of the Results object with all tensors on CPU memory.
    #     numpy(): Returns a copy of the Results object with all tensors as numpy arrays.
    #     cuda(): Returns a copy of the Results object with all tensors on GPU memory.
    #     to(*args, **kwargs): Returns a copy of the Results object with tensors on a specified device and dtype.
    #     new(): Returns a new Results object with the same image, path, and names.
    #     plot(...): Plots detection results on an input image, returning an annotated image.
    #     show(): Show annotated results to screen.
    #     save(filename): Save annotated results to file.
    #     verbose(): Returns a log string for each task, detailing detections and classifications.
    #     save_txt(txt_file, save_conf=False): Saves detection results to a text file.
    #     save_crop(save_dir, file_name=Path("im.jpg")): Saves cropped detection images.
    #     tojson(normalize=False): Converts detection results to JSON format.
