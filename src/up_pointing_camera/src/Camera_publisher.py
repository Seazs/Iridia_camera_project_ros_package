#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2

class CameraPublisher:
    def __init__(self):
        try:
            self.image_pub = rospy.Publisher("image_raw", Image, queue_size=1) # qeueu size of 1 as we are only interested in the most recent image
            self.bridge = CvBridge()
            self.cap = cv2.VideoCapture("/home/alex/catkin_ws/src/up_pointing_camera/src/video.mp4")
            rospy.loginfo("Camera has been started")
            self.rate = rospy.Rate(30)
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
            rospy.signal_shutdown("Error: {}".format(e))

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(image_message)
                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_publisher')
    rospy.loginfo("Camera Publisher Node has been started")
    #print working directory
    rospy.loginfo("Current working directory: {}".format(os.getcwd()))
    camera_publisher = CameraPublisher()
    camera_publisher.run()
    rospy.spin()