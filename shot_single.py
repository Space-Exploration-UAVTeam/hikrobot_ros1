#!/usr/bin/env python
#coding=utf-8
import rospy
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def shot_single():
    rospy.init_node("shot_single")
    rospy.loginfo( 'Taking one pictures from camera!')
    data_1 = rospy.wait_for_message('/hikrobot/image_left', Image)
    bridge = CvBridge()
    cv_img_1 = bridge.imgmsg_to_cv2(data_1, "bgr8")
    string_1 = "/home/zbh/captured"+str(rospy.Time.now())+".png"
    cv2.imwrite(string_1,cv_img_1)
    rospy.loginfo( 'pictures saved!')


if __name__=="__main__":
    shot_single()
