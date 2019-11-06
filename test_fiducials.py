#!/usr/bin/env python
from __future__ import print_function

import rospy

import sys

import actionlib

from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray,FiducialTransform
from actionlib_msgs.msg import GoalStatusArray,GoalStatus
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

import numpy as np

class Test_Fiducials:
    def __init__(self):
        rospy.init_node('test_fiducials', anonymous=False)
        

    def fiducial_callback(self,msg):
        if msg.transforms:

            # Get closest fiducial
            closest_fiducial = sorted(msg.transforms,key=lambda i: np.linalg.norm(np.array([i.transform.translation.x,
                i.transform.translation.y,i.transform.translation.z])))[0]

            pose_st = PoseStamped()
            pose_st.pose.position = closest_fiducial.transform.translation
            pose_st.pose.orientation = closest_fiducial.transform.rotation
            pose_st.header.frame_id = msg.header.frame_id
            pose_st.header.stamp = msg.header.stamp

            result = self.fiducials_client(pose_st,msg)

    def fiducials_client(self, pose_st,msg):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = pose_st
        
        client.send_goal(goal)

        print ("Goal published\nPose:\n",pose_st)
        self.sub_once.unregister()

        client.wait_for_result()
        if client.get_state() == GoalStatus.SUCCEEDED:
            print("Goal reached")
            self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
        
        return client.get_result()


    def test(self):

        self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
        while not rospy.is_shutdown():
            rospy.sleep(0.05) 


if __name__ == '__main__':
    try:
        t = Test_Fiducials()
        t.test()

    except rospy.ROSInterruptException:
    	print("Script interrupted", file=sys.stderr)
