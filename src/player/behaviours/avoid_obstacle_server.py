#!/usr/bin/env python

import rospy
import random
import actionlib

from player.msg import *


class AvoidObstacleServer:

    feedback = AvoidObstacleFeedback()
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer("avoid_obstacle", AvoidObstacleAction, self.execute, False)
        self.server.start()
        self.move_pub = rospy.Publisher("cmd_move", CmdMove,queue_size=10)
        rospy.loginfo("initialised avoid obstacle server")

    def check_preempt(self):
        if self.server.is_preempt_requested():
            rospy.loginfo("I was preempted")
            self.server.set_preempted()
            return True
        
    def execute(self, goal):
        rospy.loginfo("avoiding obstacle")
        result = AvoidObstacleResult()

        movement = CmdMove()
        movement.direction = "cw"
        movement.duration = 1
        movement.speed = 0.5
        self.move_pub.publish(movement)

        result.message = "successfully avoided"
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("avoid_obstacle_server")
    server = AvoidObstacleServer()
    
    rospy.spin()
