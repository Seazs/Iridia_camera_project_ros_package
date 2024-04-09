#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results



class CameraListener:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)
        #self.door_detection_pub = rospy.Publisher("door_detection", , queue_size=1)
        self.bridge = CvBridge()
        self.image = None
        self.yolo = YOLO("/home/alex/catkin_ws/src/up_pointing_camera/src/yolov8n.pt")
        self.classes = self.yolo.names #list of classes

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #apply the model
        result: Results = self.yolo.predict(cv_image, show= False)[0]
        for obj in result.boxes.cls: #cls is the class of the object
            detected_class = self.classes[int (obj)]
            print(detected_class)
            rospy.loginfo(detected_class)
            # data_analyzed = {"class": detected_class, "confidence": result.probs.confidence[int(obj)]}
            # self.door_detection_pub.publish(data_analyzed)
        
        cv2.waitKey(3)


if __name__ == '__main__':
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