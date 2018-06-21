#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from player.msg import *
from player import rosprint

class MoveServer:
    feedback = player.msg.MoveFeedback()
    result = player.msg.MoveResult()

    def __init__(self):
        rosprint("Initialised move server!")
        self.server = actionlib.SimpleActionServer("move", MoveAction, self.execute, False)
        self.server.start()
        #publishers
        self.move_pub = rospy.Publisher("cmd_move", CmdMove, queue_size=1000)

    #callbacks
    def execute(self, goal):
        r = rospy.Rate(1)
        sucess = True
        #publish info to console
        rosprint("Executing move in %s dir with %i duration and %i speed." % (goal.direction, goal.duration, goal.speed))
        #execute action
        self.move(goal.direction, goal.duration, goal.speed)

        r.sleep()
        #publish result
        if success:
            self.result.message = "succeeded"
            rosprint("Move successfuly executed!")
            self.server.set_succeeded(self.result)

    #move functions
    def move(self, direction, duration=0.5, speed=0.1):
        """
        Sends CmdMove message to cmd_move topic. Direction: "fwd" = forward, "cw" = clockwise, "ccw" = counterclockwise, "stop" = stop. Duration in seconds. Speed in m/s.
        """
        msg = CmdMove()
        if direction in ["fwd","cw","ccw","stop"]:
            msg.direction = direction
            if direction == "stop":
                self.move_pub.publish(msg)
        else:
            raise ValueError("Invalid direction specifier! (Valid specifiers: fwd, cw, ccw, stop)\n")
        if duration > 0:
            msg.duration = duration
        else:
            raise ValueError("Duration is negative, 0 or unspecified!\n")
        if speed > 0:
            msg.speed = speed
        else:
            raise ValueError("Speed is negative, 0 or unspecified!\n")
        self.move_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("move_server")
    build_map_server = MoveServer()

    rospy.spin()
