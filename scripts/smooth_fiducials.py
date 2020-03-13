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
LAUNCH_DISTANCE = 2.0

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

		self.fid_transform = None
		self.fid_action = None

		self.positionlist = []

		self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

		#self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		
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

					self.fid_subscriber.unregister()

					t = TransformStamped()
					t.child_frame_id = "fid%d"% fiducials[0].fiducial_id
					t.header.frame_id = "raspicam"
					t.header.stamp = msg.header.stamp
			
					t.transform.translation = fiducials[0].transform.translation
					t.transform.rotation = fiducials[0].transform.rotation

					self.broadcaster.sendTransform(t)

					print("---- Transform sent. ----")

					try:
						transform = self.buffer.lookup_transform("odom", t.child_frame_id, t.header.stamp, rospy.Duration(1.0))
						if math.fabs(self.get_marker_error(transform)) < 10:
							self.fid_transform = transform
							self.fid_action = self.fids[fiducials[0].fiducial_id]
							self.new_fid_goal()
						else:
							self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
						print("------------- MARKER ERROR: "+str(self.get_marker_error(transform))+" ------------")
					except:
						print("Couldn't find transform.")
						self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
			
	def get_marker_error(self, trans):
		r = trans.transform.rotation
		quat_rotat = (r.x, r.y, r.z, r.w)
		(roll, pitch, yaw) = euler_from_quaternion(quat_rotat)
		return math.degrees(roll) + math.degrees(pitch)

	def get_yaw(self, trans):
		r = trans.transform.rotation
		quat_rotat = (r.x, r.y, r.z, r.w)
		(roll, pitch, yaw) = euler_from_quaternion(quat_rotat)
		return yaw + math.radians(90)

	def reproject_transform(self, distance):
		x = self.fid_transform.transform.translation.x
		y = self.fid_transform.transform.translation.y
		yaw = self.get_yaw(self.fid_transform)

		x += distance * math.cos(yaw)
		y += distance * math.sin(yaw)

		self.fid_transform.transform.translation.x = x
		self.fid_transform.transform.translation.y = y

	def process_goal(self, trans, stamp):

		startPose = PoseStamped()
		startPose.header.frame_id = "odom"
		startPose.header.stamp = stamp
		
		startPose.pose.position = trans.transform.translation
		startPose.pose.position.z = 0
		
		final_rotat = quaternion_from_euler(0, 0, self.get_yaw(trans))
		startPose.pose.orientation.x = final_rotat[0]
		startPose.pose.orientation.y = final_rotat[1]
		startPose.pose.orientation.z = final_rotat[2]
		startPose.pose.orientation.w = final_rotat[3]

		goal = MoveBaseGoal()
		goal.target_pose = startPose
		self.client.send_goal(goal)


	def new_fid_goal(self):
		if self.fid_action == "GO":
			self.reproject_transform(0.25)

		self.process_goal(self.fid_transform, rospy.Time.now())

	def new_goal(self):
		self.fid_subscriber = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

		self.reproject_transform(LAUNCH_DISTANCE)
		self.process_goal(self.fid_transform, rospy.Time.now())
		print("Next goal published")

	def distance_to_base(self):
		try:
			pose = self.buffer.lookup_transform("odom", "base_link", rospy.Time.now(), rospy.Duration(0.5))
			dist = (self.fid_transform.transform.translation.x - pose.transform.translation.x) ** 2 
			dist += (self.fid_transform.transform.translation.y - pose.transform.translation.y) ** 2 
			dist = math.sqrt(dist)
			return dist
		except:
			return -1

if __name__ == '__main__':
	try:
		t = GroundFiducials()
		sleep_time = 1.0/UPDATE_RATE
		while not rospy.is_shutdown():
			rospy.sleep(sleep_time)

			if t.fid_transform != None and t.fid_action == "GO":
				dist = t.distance_to_base()
				print("DIST:"+str(dist))
				if dist > 0 and dist < 0.65:
					t.new_goal()


	except rospy.ROSInterruptException:
		print("Script interrupted", file=sys.stderr)
