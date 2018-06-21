#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient

from player.msg import *
from tools import rosprint

def done_cb(state, result):
#    rosprint("Action done: build_map, finished in state {} with result {}".format(state,result))
    pass

def active_cb():
#    rosprint("Action is now active: build_map")
    pass

def feedback_cb(feedback):
#    rosprint("Feedback build_map from action: {}".format(feedback))
    pass

if __name__ == '__main__':
    rospy.init_node("build_map_client")
    build_map_client = SimpleActionClient("build_map", BuildMapAction)
#    rosprint("Waiting for build_map server")
    build_map_client.wait_for_server()
#    rosprint("Sent data to build_map server")
    build_map_goal = BuildMapGoal()
    build_map_client.send_goal(build_map_goal, done_cb=done_cb,
            active_cb=active_cb, feedback_cb=feedback_cb)
    build_map_client.wait_for_result()
