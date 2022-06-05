#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
import tf
from tf import TransformListener, TransformerROS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest
from nav_msgs.msg import Path
import datetime
import csv
class WheelTF(object):
	def __init__(self):
		self.listener = TransformListener()
		self.pub_pose = rospy.Publisher("truth_map_posestamped", PoseStamped, queue_size=1)
		self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_timer)
		self.pub_path = rospy.Publisher('path', Path, queue_size=1)
		self.path = Path()
		self.path.header.frame_id = 'map'
		self.count = 0
	def cb_timer(self,event):
		try:
			(trans,rot) = self.listener.lookupTransform('map','base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)as e:
			print(e)
			return
		gripper_pose = PoseStamped()
		gripper_pose.header.frame_id = "base_link"
		gripper_pose.pose.position.x = 0
		gripper_pose.pose.position.y = 0
		gripper_pose.pose.position.z = 0
		gripper_pose.pose.orientation.x = 0
		gripper_pose.pose.orientation.y = 0
		gripper_pose.pose.orientation.z = 0
		gripper_pose.pose.orientation.w = 1
		base_link_gripper_pose = self.listener.transformPose('map', gripper_pose)
		base_link_gripper_pose.header.stamp = rospy.Time.now()
		base_link_gripper_pose.header.frame_id = "map"
		self.pub_pose.publish(base_link_gripper_pose)
		rospy.loginfo("pub posestamp")

		rospy.loginfo('pose marker%d' % self.count)
		self.path.header.stamp = rospy.Time.now()

		pose = PoseStamped()
		self.path.poses.append(base_link_gripper_pose)
		self.pub_path.publish(self.path)
		self.count += 1
		print("pub path",len(self.path.poses))


if __name__ == "__main__":
	rospy.init_node("drawing_path")
	wheeltf = WheelTF()
	rospy.spin()
