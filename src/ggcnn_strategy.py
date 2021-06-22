#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from moveit_commander.conversions import pose_to_list
from interbotix_sdk.msg import SingleCommand
from ggcnn_humanseg_ros.msg import GraspPrediction
import tf

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MedicalMoveGroup(object):
	def __init__(self):
		super(MedicalMoveGroup, self).__init__()

		joint_state_topic = ['joint_states:=/vx300s/joint_states']
		moveit_commander.roscpp_initialize(joint_state_topic)
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('ggcnn_strategy', anonymous=True)

		## Get the name of the robot - this will be used to properly define the end-effector link when adding a box
		self.robot_name = rospy.get_param("~robot_name","vx300s")


		## Get the dof of the robot - this will make sure that the right number of joints are controlled
		##dof = rospy.get_param("~dof")
		self.dof = 6

		## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
		## the robot:
		robot = moveit_commander.RobotCommander(robot_description="vx300s/robot_description")

		## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
		## to the world surrounding the robot:
		scene = moveit_commander.PlanningSceneInterface(ns="vx300s")

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to one group of joints.  In this case the group is the joints in the Interbotix
		## arm so we set ``group_name = interbotix_arm``. If you are using a different robot,
		## you should change this value to the name of your robot arm planning group.
		## This interface can be used to plan and execute motions on the Interbotix Arm:
		group_name = "interbotix_arm"
		group = moveit_commander.MoveGroupCommander(robot_description="vx300s/robot_description", ns="vx300s", name=group_name)

		## We create a `DisplayTrajectory`_ publisher which is used later to publish
		## trajectories for RViz to visualize:
		display_trajectory_publisher = rospy.Publisher("move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		## END_SUB_TUTORIAL

		## BEGIN_SUB_TUTORIAL basic_info
		planning_frame = group.get_planning_frame()
		print("============ Reference frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = group.get_end_effector_link()
		print("============ End effector: %s" % eef_link) 

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()
		print("============ Robot Groups:", robot.get_group_names()) 

		# robot:
		print("============ Printing robot state") 
		print(robot.get_current_state()) 
		print("") 
		## END_SUB_TUTORIAL

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.eef_link = eef_link
		self.group_names = group_names
		
		# Home
		self.home_joint_angle = [0.003067961661145091, -1.6919808387756348, -1.5707963705062866, -0.02454369328916073, -0.1457281857728958, 0.05982525274157524]
		
		# Upper layer
		self.upper_joint_angle = [0.04295146465301514, -1.104466199874878, -0.6626797318458557, -0.08590292930603027, -0.46019425988197327, 0.07976700365543365]
		
		# Pick object from human hand
		self.obj_pose_msgs = HandObjectPose()

		# service
		self.srv_home = rospy.Service('arm_home', Trigger, self.cb_home_srv)
		self.srv_upper = rospy.Service('arm_upper', Trigger, self.cb_upper_srv)
		self.srv_open = rospy.Service('arm_open', Trigger, self.cb_open_srv)
		self.srv_close = rospy.Service('arm_close', Trigger, self.cb_close_srv)
		self.srv_handover = rospy.Service('handover', Trigger, self.cb_handover)

		# subscriber
		rospy.Subscriber("/ggcnn_humanseg", GraspPrediction, self.callback, queue_size = 1)

		# publisher
		self.pub_joint_command = rospy.Publisher("/vx300s/single_joint/command", SingleCommand, queue_size=1)
		
		# Ready
		print("============ Ready ============") 

	def callback(self, obj_msg):
		self.object_pose_msgs = obj_msg

	def cb_close_srv(self, req):
 
		# Gripper 
		cmd = SingleCommand()
		cmd.joint_name = "gripper"
		cmd.cmd = 0.1
		self.pub_joint_command.publish(cmd)
		rospy.sleep(0.5)

		res = TriggerResponse()
		res.success = True
		return res

	def cb_open_srv(self, req):
 
		# Gripper 
		cmd = SingleCommand()
		cmd.joint_name = "gripper"
		cmd.cmd = 1.2
		self.pub_joint_command.publish(cmd)
		rospy.sleep(0.5)

		res = TriggerResponse()
		res.success = True
		return res

	def cb_home_srv(self, req):
		group = self.group

		joint_goal = group.get_current_joint_values()
		joint_goal = self.home_joint_angle

		group.go(joint_goal, wait=True)
		group.stop()

		res = TriggerResponse()
		res.success = True
		return res

	def cb_upper_srv(self, req):
		group = self.group

		joint_goal = group.get_current_joint_values()
		joint_goal = self.upper_joint_angle

		group.go(joint_goal, wait=True)
		group.stop()

		res = TriggerResponse()
		res.success = True
		return res


	def cb_handover(self,req):
	
		res = TriggerResponse()

		# Algorithm
		group = self.group
		robot_name = self.robot_name
		target_pose = geometry_msgs.msg.Pose() 

		# Record Pose
		target_pose.position.x = self.object_pose_msgs.pose.position.x - 0.14
		target_pose.position.y = self.object_pose_msgs.pose.position.y + 0.03
		target_pose.position.z = self.object_pose_msgs.pose.position.z + 0.06
		target_pose.orientation.x = 0
		target_pose.orientation.y = self.object_pose_msgs.pose.orientation.y
		target_pose.orientation.z = 0
		target_pose.orientation.w = self.object_pose_msgs.pose.orientation.w

		group.set_pose_target(target_pose)
		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		# rospy.sleep(0.5)

		# target_pose.position.x = target_pose.position.x +0.05
		# target_pose.position.y = target_pose.position.y
		# target_pose.position.z = target_pose.position.z
		# target_pose.orientation.x = target_pose.orientation.x
		# target_pose.orientation.y = target_pose.orientation.y
		# target_pose.orientation.z = target_pose.orientation.z
		# target_pose.orientation.w = target_pose.orientation.w 

		# group.set_pose_target(target_pose)
		# plan = group.go(wait=True)
		# group.stop()
		# group.clear_pose_targets()
		# rospy.sleep(0.3)

		cmd = SingleCommand()
		cmd.joint_name = "gripper"
		cmd.cmd = 0.5
		self.pub_joint_command.publish(cmd)
		rospy.sleep(1.5)


		res = TriggerResponse()
		res.success = True

		return res    


if __name__ == '__main__':
	medical_move_group = MedicalMoveGroup()
	rospy.spin()






