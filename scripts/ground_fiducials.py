#!/usr/bin/env python
from __future__ import print_function

import rospy

import sys

import actionlib

from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from fiducial_msgs.msg import FiducialTransformArray,FiducialTransform
from actionlib_msgs.msg import GoalStatusArray,GoalStatus
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

import tf2_ros

import numpy as np

import traceback

from tf.transformations import quaternion_from_euler
import math

ROTATION_ANGLE = 30

class GroundFiducials:
    def __init__(self):
        rospy.init_node('test_fiducials', anonymous=False)
        self.state = 'SEARCH'
        self.buffer = tf2_ros.Buffer(rospy.Time(30))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.visited_fiducials = []
        self.closest_fiducial = None

       	self.GO_fids = rospy.get_param("~GO_fiducials", ["fid49","fid51"])
       	self.STOP_fids = rospy.get_param("~STOP_fiducials", ["fid50"])

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        self.num_rotations = 0
        self.wait_for_stop = 0.0

        self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

        
    def rotate(self):

      	pose_st = PoseStamped()
        q_rot = quaternion_from_euler(0, 0, -math.radians(ROTATION_ANGLE))
      	pose_st.pose.position.x = 0
      	pose_st.pose.position.y = 0
        pose_st.pose.position.z = 0

        pose_st.pose.orientation.x = q_rot[0]
        pose_st.pose.orientation.y = q_rot[1]
        pose_st.pose.orientation.z = q_rot[2]
        pose_st.pose.orientation.w = q_rot[3]
        pose_st.header.frame_id = "base_link"
        pose_st.header.stamp = rospy.Time.now()

        goal = MoveBaseGoal()
        goal.target_pose = pose_st
        self.rotate_goal = self.client.send_goal(goal)
        self.client.wait_for_result()

            
    def fiducial_callback(self,msg):
        if msg.transforms:
            fiducials = sorted([f for f in msg.transforms if f.fiducial_id not in self.visited_fiducials],key=lambda i: np.linalg.norm(np.array([i.transform.translation.x,
                i.transform.translation.y,i.transform.translation.z])))

            if fiducials:
                if self.state == 'SEARCH' or (self.state == 'ROTATION' and (self.client.get_result() and self.client.get_state() == GoalStatus.SUCCEEDED)):
                    self.closest_fiducial = fiducials[0]

                    t = TransformStamped()
                    t.child_frame_id = "fid%d"% self.closest_fiducial.fiducial_id
                    t.header.frame_id = "base_link"
                    t.header.stamp = rospy.Time.now()
            
                    t.transform.translation = self.closest_fiducial.transform.translation
                    t.transform.rotation = self.closest_fiducial.transform.rotation

                    self.broadcaster.sendTransform(t)

                    try:
            	        tf = self.buffer.lookup_transform("base_link", t.child_frame_id,t.header.stamp,rospy.Duration(1.0))
            	        pose_st = PoseStamped()
            	        pose_st.pose.position = tf.transform.translation
                        pose_st.pose.orientation = tf.transform.rotation

            	        pose_st.header.frame_id = msg.header.frame_id
            	        pose_st.header.stamp = msg.header.stamp
            	
            	        result = self.fiducials_client(pose_st,msg)


                    except:
                        traceback.print_exc()
                        print ("Could not get transform for %s" % self.closest_fiducial.fiducial_id)
            
            else:
                print ("Can't find any fiducials. Finishing the script...")
                self.state = 'FINISHING'
            	

    def fiducials_client(self, pose_st,msg):
        goal = MoveBaseGoal()
        goal.target_pose = pose_st
        self.client.send_goal(goal)
      
        print ("Goal published\n", self.closest_fiducial)
        self.sub_once.unregister()
        self.state = 'MOVING_TO_FID'
        self.num_rotations = 0

        success = self.client.wait_for_result()

        if success and self.client.get_state() == GoalStatus.SUCCEEDED:
            print("Goal reached")
            self.visited_fiducials.append(self.closest_fiducial.fiducial_id)

            self.state = 'SEARCH'
            self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
            self.closest_fiducial = None
            
        else:
        	self.client.cancel_goal()
        
        return self.client.get_result()


    def test(self):
        sleep_time = 0.05
        while not rospy.is_shutdown():
            rospy.sleep(sleep_time)
            if self.state == 'SEARCH':
                if self.wait_for_stop<0.2:
                    self.wait_for_stop+=sleep_time
                else:
                    self.wait_for_stop = 0.0            	
            	    self.state = 'ROTATION'
            	    self.rotate()
            elif self.state == 'ROTATION':
                if self.client.get_result() and self.client.get_state() == GoalStatus.SUCCEEDED:
                    self.state = 'SEARCH'
            	    if self.num_rotations > 360.0/ROTATION_ANGLE:
                        print("Can't find any fiducials. Finishing the script...")
                        self.state = 'FINISHING'
                    else:
                        self.num_rotations+=1
                        self.rotate()
            elif self.state == 'FINISHING':
                break

if __name__ == '__main__':
    try:
        t = GroundFiducials()
        t.test()

    except rospy.ROSInterruptException:
    	print("Script interrupted", file=sys.stderr)
