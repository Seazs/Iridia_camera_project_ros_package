#!/usr/bin/env python3
import rospy



if __name__ == '__main__':
    # Initiate a Node named 'my_first_node'
    rospy.init_node('test_node')

    rospy.loginfo("Test_node has been started")
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello World!")
        rate.sleep()
    