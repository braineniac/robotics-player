#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient

from player.msg import *
from tools import rosprint

def move_client(direction="fwd", duration=1, speed=1):
    #setup client
    client = actionlib.SimpleActionClient("move", MoveAction)
    client.wait_for_server()
    #cerate and send goal
    goal = player.msg.MoveGoal()
    goal.direction = direction
    goal.duration = duration
    goal.speed = speed
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node("move_client")
        result = move_client()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion.", file=sys.stderr)
