#!/usr/bin/env python3

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

def open_gripper():
    rospy.init_node('open_gripper_client')

    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    rospy.loginfo("Waiting for /franka_gripper/grasp action server...")
    client.wait_for_server()

    goal = GraspGoal()
    goal.width = 0.08         # fully open gripper (max ~8 cm)
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 0.0          # force 0 means just move to position without grasping

    rospy.loginfo("Sending grasp goal to open gripper...")
    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo("Gripper is now open.")

if __name__ == '__main__':
    open_gripper()
