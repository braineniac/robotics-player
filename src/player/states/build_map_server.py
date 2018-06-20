#!/usr/bin/env python

import rospy
import actionlib
import numpy as np

from player.msg import *
from player import rosprint

class BuildMapServer:

    def __init__(self):
        rosprint("Initialised build_map server!")
        self.server = actionlib.SimpleActionServer("build_map", BuildMapAction, self.execute, False)
        self.server.start()
        self.feedback = BuildMapFeedback()
        self.scanned_objs_sub = rospy.Subscriber("kinect_objs", KinectObjs, self.det_objs_cb)
        self.map_init = False
        self.det_objs = None
        self.poles = None

#callbacks

    def det_objs_cb(self, det_objs_msg):
        self.det_obj = det_objs_msg

#map functions

    def look_for_poles(self):
        i = 0
        if self.det_objs:
            for det_obj in self.det_objs:
                if mapped_obj.color == "G":
                    self.poles.append(det_obj)
            if len(self.poles) >= 3:
                return True
            else:
                return False
        else:
            return False

    def dist_to_robot(self, pole):
        if pole is not None:
            x = pole.x
            y = pole.y
            d = np.sqrt(x**x + y**y)
            return d

    def get_closest_poles(self):
        pass

    def dist_two_poles(self, pole0, pole1):
        d = 0
        z0 = dist_to_robot(pole0)
        z1 = dist_to_robot(pole1)
        phi = np.arctan(pole0.y/pole0.x) - np.arctan(pole1.y/pole1.x)
        d = np.sqrt(z0*z0 + z1*z1 -2*z0*z1*np.cos(phi))
        return d

    def get_map_unit(self, close_poles):
        if close_poles is not None:
            map_unit = 0
            d1 = self.dist_two_poles(close_poles[0],close_poles[1])
            d2 = self.dist_two_poles(close_poles[1], close_poles[2])
            if d1 or d2:
                return map_unit
            elif d1/d2 - 2/3 < 0.01:
                map_unit = d2*6.6666/5
            elif d1/d2 - 3/5 < 0.01:
                map_unit = d2 * 4/5
            elif d1/d2 - 1/2 < 0.01:
                map_unit = d2 * 4 / 5
            elif d1/d2 - 0.5/3 - 0.01:
                map_unit = d2/3
            return map_unit

    def build_map(self):
        close_poles = self.get_closest_poles()
        map_unit = self.get_map_unit(close_poles)
        if map_unit:
            # publish to a map service
            pass

#server functions

    def check_preemt(self):
        if self.server.is_preempt_requested():
            #rosprint("Preemtied: build_map")
            self.server.set_preempted()
            return True

    def execute(self, goal):
        #rosprint("Goal: build_map")
        result = BuildMapResult()
        if self.look_for_poles() is True:
            if self.build_map() is True:
                self.map_init = True

        self.feedback.message = "something"
        self.server.publish_feedback(self.feedback)
        if self.map_init is True:
            result.message = "build_map_successeded"
            self.server.set_succeeded(result)
        else:
            result.message = "build_map_failed"
            self.server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node("build_map_server")
    build_map_server = BuildMapServer()

    rospy.spin()

