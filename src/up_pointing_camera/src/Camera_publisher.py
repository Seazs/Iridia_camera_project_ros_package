#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import os
import cv2

class CameraPublisher:
    def __init__(self):
        try:
            self.image_pub = rospy.Publisher("image_raw", Image, queue_size=1) # qeueu size of 1 as we are only interested in the most recent image
            self.bridge = CvBridge()
            self.cap = cv2.VideoCapture(0)
            rospy.loginfo("Camera has been started")
            self.rate = rospy.Rate(1)
            
            self.image_received = True
            self.image_sub = rospy.Subscriber("image_proccesed_confirmation", Empty, self.image_callback, queue_size=1)
            rospy.loginfo("Camera subscriber has been started")
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
            rospy.signal_shutdown("Error: {}".format(e))

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if ret and self.image_received:
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(image_message)
                rospy.loginfo("Image has been published")
                self.image_received = False
            self.rate.sleep()
    
    def image_callback(self, data):
        rospy.loginfo("Image has been received")
        self.image_received = True
        

if __name__ == '__main__':
    rospy.init_node('camera_publisher')
    rospy.loginfo("Camera Publisher Node has been started")
    #print working directory
    rospy.loginfo("Current working directory: {}".format(os.getcwd()))
    camera_publisher = CameraPublisher()
    camera_publisher.run()
    rospy.spin()