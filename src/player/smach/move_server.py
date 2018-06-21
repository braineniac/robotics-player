#! /usr/bin/env python

import rospy
import actionlib
import numpy as np

from player.msg import *
from player import rosprint

class MoveServer:
    feedback = player.msg.MoveFeedbck()
    result = player.msg.MoveResult()

    def __init__(self):
        rosprint("Initialised move server!")
        self.server = actionlib.SimpleActionServer("move", MoveAction, self.execute, False)
        self.server.start()
        #publishers
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)

    #callbacks
    def execute(self, goal):
        sucess = True
        #publish info to console
        rosprint("Executing move in %s dir with %i duration and %i speed." % (goal.direction, goal.duration, goal.speed))
        #execute action
        self.move(goal)
        #publish result
        if success:
            self.result.message = "succeeded"
            rosprint("Move successfuly executed!")
            self.server.setset_succeeded(self.result)

    #move functions
    def move(self, goal):
        timeLeft = goal.duration
        while timeLeft > 0:
            if goal.direction.lower() == "fwd":
                self.forward(goal.speed)
            elif goal.direction.lower() == "cw":
                self.turnRight(goal.speed)
            elif goal.direction.lower() == "ccw":
                self.turnLeft(goal.speed)
            elif goal.direction.lower() == "stop":
                self.stop()
            else:
                raise ValueError("Invalid direction specifier! (Valid specifiers: fwd, cw, ccw, stop)\n")
            if timeLeft >= 0.25:
                rospy.sleep(0.25)
                timeLeft -= 0.25
            else:
                rospy.sleep(timeLeft)
                timeLeft = 0
            #if preempted
            if self.server.is_preempt_requested():
                rosprint("Move preempted!")
                self.server.set_preempted()
                success = False
                break
            #feedback
            self.feedback.time_left = timeLeft
            self.server.publish_feedback(self.feedback)

    def forward(self,speed=0):
        if speed > 0:
            self.__set_velocities(speed, 0)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def turnRight(self,speed=0):
        if speed > 0:
            self.__set_velocities(0, -speed)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def turnLeft(self,speed=0):
        if speed > 0:
            self.__set_velocities(0, speed)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def stop(self):
        self.__set_velocities(0,0)

    def __set_velocities(self,linear=0, angular=None):
        if linear >= 0 and angular is not None:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.vel_pub.publish(msg)
        else:
            exit(-1)

if __name__ == '__main__':
    rospy.init_node("move_server")
    build_map_server = MoveServer()

    rospy.spin()
