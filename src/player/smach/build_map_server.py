#!/usr/bin/env python

import rospy
import actionlib
import sys
from player.msg import *
from player import rosprint

class BuildMapServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer("build_map", BuildMapAction, self.execute, False)
        self.server.start()
        self.feedback = BuildMapFeedback()
        rosprint("Initialised build_map server!")

    def check_preemt(self):
        if self.server.is_preempt_requested():
            rosprint("Preemtied: build_map")
            self.server.set_preempted()
            return True

    def execute(self, goal):
        rosprint("Goal: build_map")
        result = BuildMapResult()
        self.feedback.message = "something"
        self.server.publish_feedback(self.feedback)
        result.message = "build_map_failed"
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("build_map_server")
    build_map_server = BuildMapServer()

    rospy.spin()


