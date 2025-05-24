#!/usr/bin/env python3

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

def close_gripper():
    rospy.init_node('close_gripper_client')

    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    rospy.loginfo("Waiting for /franka_gripper/grasp action server...")
    client.wait_for_server()

    goal = GraspGoal()
    goal.width = 0.0          # fully closed gripper
    goal.epsilon.inner = 0.005
    goal.epsilon.outer = 0.005
    goal.speed = 0.1
    goal.force = 10.0         # grasp force

    rospy.loginfo("Sending grasp goal to close gripper...")
    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo("Gripper action finished.")

if __name__ == '__main__':
    close_gripper()
