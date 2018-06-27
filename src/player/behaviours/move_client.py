#!/usr/bin/env python

import rospy
import actionlib

from player.msg import *
from player import rosprint

def move_client():
    #setup client
    client = actionlib.SimpleActionClient("move", MoveAction)
    client.wait_for_server()
    #cerate and send goal
    move_goal = MoveGoal()
    move_goal.direction = "ccw"
    move_goal.duration = 1
    move_goal.speed = 0.5
    client.send_goal(move_goal)

    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node("move_client")
        result = move_client()
    except rospy.ROSInterruptException:
        rosprint("Program interrupted before completion.")
