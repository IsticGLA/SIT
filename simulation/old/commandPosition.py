#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Command:
	def __init__(self):
		self.cmd = rospy.Publisher("/waypoint", Pose, queue_size=10, latch=True)
		rospy.init_node("cat_node")

	def setWaypoint(self, x, y, z):
		pose = Pose(position=Point(x,y,z))
		self.cmd.publish(pose)

if __name__ == "__main__":
	command = Command()
	while(True):
		x = input("x? ")
		y = input("y? ")
		z = input("z? ")
		command.setWaypoint(x,y,z)
