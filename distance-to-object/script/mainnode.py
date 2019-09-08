#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import message_filters

bridge = cv_bridge.CvBridge()


def get_color_area(frame):
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	LOW_COLOR = np.array([100, 75, 75])
	HIGH_COLOR = np.array([140, 255, 255])
	mask = cv2.inRange(hsv, LOW_COLOR, HIGH_COLOR)
	cv2.imshow("mask", mask)
	image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	areas = np.array(list(map(cv2.contourArea,contours)))
	if len(areas) == 0 or np.max(areas) / (640*480) < 0.005:
        	print("the area is too small")
		return None
	max_idx = np.argmax(areas)
	#max_area = areas[max_idx]
	max_area = contours[max_idx]
	
	ellipse = cv2.fitEllipse(contours[max_idx])
	im = cv2.ellipse(frame,ellipse,(0,255,0),2)
	
	return max_area

def callback(color_msgs, depth_msgs):
	try:
		image = bridge.imgmsg_to_cv2(color_msgs, 'bgr8')
		depth = bridge.imgmsg_to_cv2(depth_msgs, '16UC1')
	except cv_bridge.CvBridgeError as e:
		rospy.logerr(e)
	
	area = get_color_area(image)
	#depth_mask = np.array(depth[area])
	rospy.loginfo(area)
	cv2.imshow("image", image)
	#cv2.imshow("depth", depth)
	cv2.waitKey(3)

def listener():
	rospy.init_node('mainnode')
	ts = message_filters.ApproximateTimeSynchronizer([message_filters.Subscriber("/camera/color/image_raw", Image), message_filters.Subscriber("/camera/depth/image_rect_raw", Image)], 15, 0.1)
	ts.registerCallback(callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException: pass
