#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import actionlib
import math
import tf2_ros
import numpy as np
import traceback

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from copy import deepcopy
from tf2_geometry_msgs import do_transform_vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion

UPDATE_RATE = 30

def arrayify(string):
	string = string.replace(' ', '').replace('[','').replace(']','')
	array = string.split(',')
	return [ int(x) for x in array ]

class GroundFiducials:
	def __init__(self):
		rospy.init_node('test_fiducials', anonymous=False)
		self.buffer = tf2_ros.Buffer(rospy.Time(30))

		self.listener = tf2_ros.TransformListener(self.buffer)
		self.broadcaster = tf2_ros.TransformBroadcaster()

		GO_fids = arrayify(rospy.get_param("~GO_fiducials", "[51, 49]"))
		STOP_fids = arrayify(rospy.get_param("~STOP_fiducials", "[50]"))

		self.fids = {}
		for x in GO_fids:
			self.fids[x] = "GO"
		for x in STOP_fids:
			self.fids[x] = "STOP"

		self.waiting = False

		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()

		self.closest_fiducial = None;

		self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

		
	# def rotate(self):

	# 	pose_st = PoseStamped()
	# 	q_rot = quaternion_from_euler(0, 0, -math.radians(ROTATION_ANGLE))
	# 	pose_st.pose.position.x = 0
	# 	pose_st.pose.position.y = 0
	# 	pose_st.pose.position.z = 0

	# 	pose_st.pose.orientation.x = q_rot[0]
	# 	pose_st.pose.orientation.y = q_rot[1]
	# 	pose_st.pose.orientation.z = q_rot[2]
	# 	pose_st.pose.orientation.w = q_rot[3]
	# 	pose_st.header.frame_id = "base_link"
	# 	pose_st.header.stamp = rospy.Time.now()

	# 	goal = MoveBaseGoal()
	# 	goal.target_pose = pose_st
	# 	self.rotate_goal = self.client.send_goal(goal)

	# 	success = self.client.wait_for_result()

	# 	if success and self.client.get_state() == GoalStatus.SUCCEEDED:
	# 		rospy.sleep(1.0)
	# 		self.state = 'SEARCH'

	# 	if self.angle == -2*ROTATION_ANGLE:
	# 		self.angle = 2*ROTATION_ANGLE
	# 	else:
	# 		self.angle = -2*ROTATION_ANGLE

	def transformVector(self, x, y, z, quat):

		v = Vector3Stamped()
		v.vector.x = x
		v.vector.y = y
		v.vector.z = z

		t = TransformStamped()
		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]

		return do_transform_vector3(v, t)

	def fiducial_callback(self,msg):

		if msg.transforms:

			fiducials = sorted([f for f in msg.transforms],key=lambda i: np.linalg.norm(np.array([i.transform.translation.x, i.transform.translation.y,i.transform.translation.z])))

			for x in fiducials:
				if x.fiducial_id not in self.fids:
					fiducials.remove(x)

			if fiducials:

					self.closest_fiducial = fiducials[0]

					print(self.closest_fiducial)

					t = TransformStamped()
					t.child_frame_id = "fid%d"% self.closest_fiducial.fiducial_id
					t.header.frame_id = "raspicam"
					t.header.stamp = msg.header.stamp
			
					t.transform.translation = self.closest_fiducial.transform.translation
					t.transform.rotation = self.closest_fiducial.transform.rotation

					self.broadcaster.sendTransform(t)

					print("---- Transform sent. ----")

					try:
						tf = self.buffer.lookup_transform("base_link", t.child_frame_id, t.header.stamp, rospy.Duration(1.0))

						self.process_goal(tf, msg.header.stamp);
					except:
						traceback.print_exc()
						print ("Could not get transform for %s" % self.closest_fiducial.fiducial_id)
			
	def process_goal(self, tf, stamp):
		# Generate the allignment pose

		startPose = PoseStamped()
		startPose.header.frame_id = "base_link"
		startPose.header.stamp = stamp

		startPose.pose.position = tf.transform.translation
		startPose.pose.position.z = 0

		rotat = tf.transform.rotation
		quat_rotat = (rotat.x, rotat.y, rotat.z, rotat.w)
		(roll, pitch, yaw) = euler_from_quaternion(quat_rotat)

		final_rotat = quaternion_from_euler(0, 0, yaw + math.radians(90))
		startPose.pose.orientation.x = final_rotat[0]
		startPose.pose.orientation.y = final_rotat[1]
		startPose.pose.orientation.z = final_rotat[2]
		startPose.pose.orientation.w = final_rotat[3]

		# Generate target pose to launch towards

		targetPose = PoseStamped()
		targetPose.header.frame_id = "base_link"
		targetPose.header.stamp = stamp
		targetPose.pose.orientation.x = 0
		targetPose.pose.orientation.y = 0
		targetPose.pose.orientation.z = 0
		targetPose.pose.orientation.w = 1
		targetPose.pose.position.x = 5.0

		print("---- GOAL SENT. ----")

		result = self.send_goal(startPose, targetPose)


	def send_goal(self, startPose, targetPose):

		self.fid_subscriber.unregister()

		goal = MoveBaseGoal()
		goal.target_pose = startPose
		self.client.send_goal(goal)

		print("Start goal published\n", self.closest_fiducial.fiducial_id)

		success = self.client.wait_for_result()

		if success and self.client.get_state() == GoalStatus.SUCCEEDED:
			print("Goal reached")

			self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
			self.closest_fiducial = None

			goal = MoveBaseGoal()
			goal.target_pose = targetPose
			self.client.send_goal(goal)

			print("Target goal published!")
			
		else:
			self.client.cancel_goal()
		
		return self.client.get_result()


	def wait(self, toggle):
		if toggle:
			self.waiting = True
			self.fid_subscriber.unregister()

		else:
			self.waiting = False


if __name__ == '__main__':
	try:
		t = GroundFiducials()
		sleep_time = 1.0/UPDATE_RATE
		while not rospy.is_shutdown():
			rospy.sleep(sleep_time)
			if t.waiting:
				get = input("Press any key to continue")
				if get:
					t.waiting = False

	except rospy.ROSInterruptException:
		print("Script interrupted", file=sys.stderr)
