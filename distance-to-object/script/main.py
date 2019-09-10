#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
import message_filters

bridge = cv_bridge.CvBridge()
matrix = np.empty((3,3))
before = None


def set_info(camera_msgs):
	global matrix
	matrix = np.array(camera_msgs.K).reshape((3, 3))

def get_color_coordinate(frame):
	x = np.zeros((2, 2))
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	#enemy move tracking
	global before
	if before is None:
		before = gray.copy().astype('float')
	cv2.accumulateWeighted(gray, before, 0.25)
	m_image = cv2.absdiff(gray, cv2.convertScaleAbs(before))
	m_mask = cv2.threshold(m_image, 30, 255, cv2.THRESH_BINARY)[1]
	m_image, m_contours, m_hierarchy = cv2.findContours(m_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	m_area = np.array(list(map(cv2.contourArea,m_contours)))
	if (len(m_area) != 0 and np.nanmax(m_area) / (640*480) > 0.005):
		m_max_idx = np.argmax(m_area)
		cv2.drawContours(frame, m_contours[m_max_idx], -1, (255,255,255), 3)

	#color_masking
	LOW_RED = np.array([175, 75, 100])
	HIGH_RED = np.array([180, 255, 255])
	LOW_BLUE = np.array([95, 75, 150])
	HIGH_BLUE = np.array([100, 255, 255])
	r_mask = cv2.inRange(hsv, LOW_RED, HIGH_RED)
	b_mask = cv2.inRange(hsv, LOW_BLUE, HIGH_BLUE)
	r_image, r_contours, r_hierarchy = cv2.findContours(r_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	b_image, b_contours, b_hierarchy = cv2.findContours(b_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	r_area = np.array(list(map(cv2.contourArea,r_contours)))
	b_area = np.array(list(map(cv2.contourArea,b_contours)))
	if (len(r_area) != 0 and np.nanmax(r_area) / (640*480) > 0.005):
		r_max_idx = np.argmax(r_area)
		r_max_area = r_contours[r_max_idx]
		r_moment = cv2.moments(r_max_area)
		x[0, 0] = int(r_moment['m10'] / r_moment['m00'])
		x[0, 1] = int(r_moment['m01'] / r_moment['m00'])
		cv2.drawContours(frame, r_contours[r_max_idx], -1, (0,0,255), 3)
		r_mask_mask = np.zeros_like(frame)
		cv2.fillConvexPoly(r_mask_mask, r_contours[r_max_idx], (255,255,255))
		r_M = np.float32([[0.5, 0, 160], [0, 0.5, 120]])
		res_r_mask_mask = cv2.warpAffine(r_mask_mask, r_M, (640, 480))
	
	if (len(b_area) != 0 and np.nanmax(b_area) / (640*480) > 0.005):
		b_max_idx = np.argmax(b_area)
		b_max_area = b_contours[b_max_idx]
		b_result = cv2.moments(b_max_area)
		x[1, 0] = int(b_result['m10'] / b_result['m00'])
		x[1, 1] = int(b_result['m01'] / b_result['m00'])
		cv2.drawContours(frame, b_contours, b_max_idx, (255,0,0), 3)
		b_mask_mask = np.zeros_like(frame)
		cv2.fillConvexPoly(b_mask_mask, b_contours[b_max_idx], (255,255,255))
		b_M = np.float32([[0.5, 0, 160], [0, 0.5, 120]])
		res_b_mask_mask = cv2.warpAffine(b_mask_mask, b_M, (640, 480))
		cv2.imshow("mask", res_b_mask_mask)
	
	return x

def calcu_scale(color_msgs, depth_msgs):
	try:
		image = bridge.imgmsg_to_cv2(color_msgs, 'bgr8')
		depth = bridge.imgmsg_to_cv2(depth_msgs, '16UC1')
	except cv_bridge.CvBridgeError as e:
		rospy.logerr(e)
	
	area = get_color_coordinate(image)
	u = int(area[1,0])
	v = int(area[1,1])
	distance = depth[v,u]
	test = np.array([(u - 319.5) / matrix[0,0] * distance, (v - 239.5) / matrix[1,1] * distance])
	
	rospy.loginfo(test)

	cv2.imshow("image", image)
	cv2.waitKey(5)


def setup():
	rospy.Subscriber("/camera/depth/camera_info", CameraInfo, set_info)

def listener():
	rospy.init_node('main')
	ts = message_filters.ApproximateTimeSynchronizer([message_filters.Subscriber("/camera/color/image_raw", Image), message_filters.Subscriber("/camera/depth/image_rect_raw", Image)], 10000, 0.5)
	ts.registerCallback(calcu_scale)
	rospy.spin()

if __name__ == '__main__':
	try:
		setup()
	except rospy.ROSInterruptException: pass
	try:
		listener()
	except rospy.ROSInterruptException: pass
