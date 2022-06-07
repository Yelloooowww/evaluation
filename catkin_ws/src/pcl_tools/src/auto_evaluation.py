#! /usr/bin/env python3
import rospy
import time
import numpy as np
import math
import random
import sys
import os
from matplotlib import pyplot as plt
from sensor_msgs.msg import LaserScan, Imu
from std_srvs.srv import Empty
from std_msgs.msg import Int64
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest
from scipy.spatial.transform import Rotation as R  #pip install scipy==1.2.0


class Evaluater(object):
	def __init__(self, csv_path, test_times, timeout, reached_dis = 1):
		self.test_times = test_times
		self.timeout = timeout
		self.csv_path = csv_path
		self.reached_dis = reached_dis
		# self.sub_laser_upper = rospy.Subscriber('/RL/scan', LaserScan, self.cb_laser, queue_size=1)
		self.reset_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
		self.pub_path = rospy.Publisher('path', Path, queue_size=1)
		self.this_time_path = []
		self.all_time_path = []
		self.path = Path()
		self.INITIAL_STATES = np.genfromtxt(self.csv_path, delimiter=',')
		print(self.INITIAL_STATES.shape)
		self.times = 0
		self.stop = False
		self.goal = [0,0,0]
		self.start = [0,0,0]

		self.success = 0
		self.TIMEOUT = 0
		self.tatal_collision = 0
		self.flag_contact = False
		self.sub_contact = rospy.Subscriber('bumper_states', ContactsState, self.cb_contact, queue_size = 1)
		self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_timer)
		self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.reset()

	def cb_contact(self, msg):
		if (msg.states == []) :self.flag_contact = False
		if (msg.states != []) and (not self.flag_contact):
			self.tatal_collision += 1
			print("!!!!!!!! collision !!!!!!!!!", self.tatal_collision)
			self.flag_contact = True


	def pause_gym(self, pause=True):
		srv_name = '/gazebo/pause_physics' if pause else '/gazebo/unpause_physics'
		rospy.wait_for_service(srv_name)
		try:
			if pause: self.pause_physics()
			else: self.unpause_physics()
		except (rospy.ServiceException) as e:
			print(e)

	def get_initial_state(self, name, init_position, goal_position):
		# start position
		state_msg = ModelState()
		state_msg.model_name = name
		state_msg.pose.position.x = init_position[0]
		state_msg.pose.position.y = init_position[1]
		state_msg.pose.position.z = init_position[2]

		# angle = random.uniform(-np.pi,np.pi)

		diff = np.array(goal_position) - np.array(init_position)
		angle = math.atan2(diff[1],diff[0])

		angle = -0.5*np.pi

		if angle >= np.pi:
			angle -= 2*np.pi
		elif angle <= -np.pi:
			angle += 2*np.pi
		r = R.from_euler('z', angle)
		quat = r.as_quat()

		state_msg.pose.orientation.x = quat[0]
		state_msg.pose.orientation.y = quat[1]
		state_msg.pose.orientation.z = quat[2]
		state_msg.pose.orientation.w = quat[3]
		print(state_msg)
		return

	def reset(self):
		print("________________RESET")
		self.stop = True
		# self.pause_gym(False)

		if not self.times == 0: self.all_time_path.append(self.this_time_path)
		self.this_time_path = []

		index = self.times if self.times<self.INITIAL_STATES.shape[0]/2 \
			else np.random.randint(low=0,high=self.INITIAL_STATES.shape[0]/2,size=1)[0]
		self.start = self.INITIAL_STATES[index*2]
		self.goal = self.INITIAL_STATES[index*2+1]
		if(random.random() > 0.5): self.start, self.goal = self.goal, self.start

		try:
			self.reset_model(self.get_initial_state('robot', self.start, self.goal))
		except(rospy.ServiceException) as e:
			print(e)

		goal_pose = PoseStamped()
		goal_pose.pose.position.x = self.goal[0]
		goal_pose.pose.position.y = self.goal[1]
		goal_pose.pose.position.z = self.goal[2]
		goal_pose.pose.orientation.x = 0
		goal_pose.pose.orientation.y = 0
		goal_pose.pose.orientation.z = 0
		goal_pose.pose.orientation.w = 1
		self.pub_goal.publish(goal_pose)


		self.path = Path()
		self.times += 1


		# self.pause_gym(True)
		self.stop = False

	def cb_timer(self,event):
		if self.stop : return

		# path
		posestamped = PoseStamped()
		try:
			posestamped = rospy.wait_for_message('truth_map_posestamped', PoseStamped, timeout=5)
		except:
			print('fail to receive message')
			return
		self.path.poses.append(posestamped)
		self.pub_path.publish(self.path)
		self.this_time_path.append([posestamped.pose.position.x, posestamped.pose.position.y])

		dis = np.linalg.norm([self.goal[0], self.goal[1]]- [posestamped.pose.position.x, posestamped.pose.position.y])
		if dis < self.reached_dis:
			rospy.loginfo("goal reached")
			self.reset()


if __name__ == "__main__":
	rospy.init_node("Evaluater")
	csv = '/home/argsubt/evaluation/catkin_ws/src/pcl_tools/src/forest_goals - test_goals.csv'
	evaluater = Evaluater(csv,5,1000)
	rospy.spin()
