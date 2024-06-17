#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int16MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results
from time import sleep
from math import tan

# Constants for camera and detection settings
CAMERA_RATIO = 12 / 9  # Aspect ratio: 2592/1944 pixels -> 640/480 pixels
HEIGHT_PX = 480
WIDTH_PX = 640
FOV = 130 * 3.14159265358979323846 / 180  # Field of View in radians
ROOM_HEIGHT = 2  # Assumed height of the room in meters
ALPHA = 2  # Exponent used in distance calculation
SHOW_IMAGE = False # show detection on image (can not be turned on on the robot)

def get_center_of_object(obj):
    """
    Calculate the center of the bounding box of an object.

    Parameters:
    obj: The object detection result with bounding box coordinates.

    Returns:
    Tuple of (center_x, center_y) representing the center of the bounding box.
    """
    x1, y1, x2, y2 = obj.xyxy[0]
    return (x1 + x2) / 2, (y1 + y2) / 2

def get_real_position_from_detection(detection):
    """
    Calculate the real-world position of a detected object based on its position in the image.

    Parameters:
    detection: The detection result including the bounding box.

    Returns:
    Tuple of (x_distance, y_distance) representing the real-world position of the object.
    """
    image_center_x, image_center_y = (WIDTH_PX / 2, HEIGHT_PX / 2)
    door_center_x, door_center_y = get_center_of_object(detection)
    dx = (door_center_x - image_center_x)
    dy = (door_center_y - image_center_y)

    x_distance = ROOM_HEIGHT * tan(FOV / 2) * (abs(dx) / (WIDTH_PX / 2)) ** ALPHA * (1 if dx > 0 else -1)
    y_distance = ROOM_HEIGHT * tan(FOV / 2) * (abs(dy) / (WIDTH_PX / 2)) ** ALPHA * (1 if dy > 0 else -1)
    return x_distance, y_distance

class CameraListener:
    """
    A ROS node for listening to camera images, detecting doors, and publishing their positions.
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.yolo = YOLO("/home/alex/catkin_ws/src/up_pointing_camera/src/best.pt")
        self.classes = self.yolo.names  # List of classes detected by the model

        # ROS Subscribers and Publishers
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=10)
        self.door_detection_pub = rospy.Publisher("door_detection", Int16MultiArray, queue_size=1)

        # This topic make the Camera_publisher node waiting before sending new image
        self.confirmation_pub = rospy.Publisher("image_proccesed_confirmation", Empty, queue_size=1)
        
        # Initial confirmation
        sleep(1)
        self.confirmation_pub.publish(Empty())

    def callback(self, data):
        """
        Callback function for image subscriber. Processes each frame to detect doors and publish their positions.

        Parameters:
        data: The image data from the ROS message.
        """
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        result: Results = self.yolo.predict(cv_image, show=SHOW_IMAGE)[0] #the prediction
        for obj in result.boxes:
            detected_class = self.classes[int(obj.cls)]
            rospy.loginfo(detected_class)
            if detected_class == "doors":
                self.process_door_detection(obj, cv_image)

        self.confirmation_pub.publish(Empty())

    def process_door_detection(self, obj, cv_image):
        """
        Process a single door detection, drawing on the image and publishing the door's position.

        Parameters:
        obj: The detected object.
        cv_image: The current frame as a CV image.
        """
        x1, y1, x2, y2 = obj.xyxy[0]
        center_x, center_y = get_center_of_object(obj)
        x_distance, y_distance = get_real_position_from_detection(obj)

        rospy.loginfo(f"center_x: {center_x}, center_y: {center_y}")
        rospy.loginfo(f"x_distance: {x_distance}, y_distance: {y_distance}")

        if SHOW_IMAGE:
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            cv2.circle(cv_image, (int(center_x), int(center_y)), 10, (0, 255, 0), -1)
            cv2.imshow("image", cv_image)

        data_analysed = Int16MultiArray(data=[int(x_distance * 100), int(y_distance * 100)])
        self.door_detection_pub.publish(data_analysed)

if __name__ == '__main__':
    rospy.init_node('camera_listener', anonymous=True)
    rospy.loginfo("Camera Listener Node has been started")
    camera_listener = CameraListener()
    rospy.spin()