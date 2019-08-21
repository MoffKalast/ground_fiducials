#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray,FiducialTransform
from actionlib_msgs.msg import GoalStatusArray,GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal

import numpy as np

class Test_Fiducials:



    def __init__(self):
        rospy.init_node('test_fiducials', anonymous=False)
        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.goal_counter = 0
        

    def goal_status_callback(self,msg):
        for m in msg.status_list:
            if m.goal_id == self.goal_id and m.status == GoalStatus.SUCCEEDED:
                print ("Goal Id {} reached".format(m.goal_id.id))
                self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
                self.goal_status.unregister()


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

            goal = MoveBaseActionGoal()
            goal.goal.target_pose = pose_st
            goal.goal_id.id = str(self.goal_counter)
            self.goal_counter+=1
            goal.goal_id.stamp = msg.header.stamp

            self.goal_id = goal.goal_id


            self.pub.publish(goal)
            print ("Goal Id {} published".format(goal.goal_id.id))
            self.goal_status = rospy.Subscriber("/move_base/status",GoalStatusArray,self.goal_status_callback)
            self.sub_once.unregister()


    def test(self):    

        self.sub_once = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)
        while not rospy.is_shutdown():
            rospy.sleep(0.05) 


if __name__ == '__main__':
    try:
        t = Test_Fiducials()
        t.test()
    except rospy.ROSInterruptException:
        pass