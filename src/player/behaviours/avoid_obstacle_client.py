#!/usr/bin/env python

import rospy
import actionlib

from player.msg import AvoidObstacleAction, AvoidObstacleGoal
from actionlib_msgs.msg import GoalStatus

def done_cb(state, result):
    rospy.loginfo("action is done, finished in state {} with result {}".format(state, result))

def active_cb():
    rospy.loginfo("action is now active")

def feedback_cb(feedback):
    rospy.loginfo("got feedback from action: {}".format(feedback))
    
if __name__ == '__main__':
    rospy.init_node("avoid_obstacle_client")
    client = actionlib.SimpleActionClient("avoid_obstacle", AvoidObstacleAction)
    rospy.loginfo("waiting for server")
    client.wait_for_server()
    rospy.loginfo("sent data to server")
    goal = AvoidObstacleGoal()
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    client.wait_for_result()
