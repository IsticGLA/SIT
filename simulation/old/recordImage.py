#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageRegister:
	def __init__(self):
		print("init")
		rospy.init_node("cat_node")
		self.subscriber = rospy.Subscriber("cat/camera/image", Image, self.process_image,  queue_size = 10)

	def process_image(self, ros_image):
		print("processing image")
		#### direct conversion to CV2 ####
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="rgb8")
		cv2.imwrite("testimage.png", cv_image)

if __name__ == "__main__":
	imageReg = ImageRegister()
	rospy.spin()
