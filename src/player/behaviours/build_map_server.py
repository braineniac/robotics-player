#!/usr/bin/env python

import rospy
import actionlib
import numpy as np

from player.msg import *
from player import rosprint
from std_msgs.msg import Bool

class BuildMapServer:

    def __init__(self):
        rosprint("Initialised build_map server!")
        self.map_init = False
        self.det_objs = None
        self.server = actionlib.SimpleActionServer("build_map", BuildMapAction, self.execute, False)
        self.server.start()
        self.feedback = BuildMapFeedback()
        self.detected_objs_sub = rospy.Subscriber("detected_objs", DetectedObjs, self.det_objs_cb)
        self.OR_pub = rospy.Publisher("OR_execution", Bool, queue_size=1)
        self.map_init = False
        self.det_objs = None


    #callbacks
    def det_objs_cb(self, det_objs_msg):
        self.det_objs = det_objs_msg

    #map functions
    def extract_poles(self):
        poles = []
        #rosprint(self.det_objs)
        if self.det_objs:
            for det_obj in self.det_objs.detectedObjList:
                if det_obj.id == "pole":
                    #rosprint("Found pole")
                    poles.append(det_obj)
        return poles

    def dist_to_robot(self, pole):
        if pole is not None:
            x = pole.x
            y = pole.y
            d = np.sqrt(x*x + y*y)
            return d

    def get_closest_poles(self, poles):
        p0 = None
        p1 = None
        p2 = None
        d1 = np.inf
        d2 = np.inf
        #finds 2 closest poles
        for pole in poles:
            other_poles = poles
            other_poles.remove(pole)
            for other_pole in other_poles:
                if self.dist_two_poles(pole, other_pole) < d1:
                    p0 = pole
                    p1 = other_pole
                    d1 = self.dist_two_poles(pole, other_pole)
        #finds 3rd closest poles
        other_poles = poles
        other_poles.remove(p0)
        other_poles.remove(p1)
        closest_from_pair = None
        for pole in other_poles:
            if self.dist_two_poles(pole, p0) < d2:
                p3 = pole
                closest_from_pair = p0
                d2 = self.dist_two_poles(pole, p0)
            if self.dist_two_poles(pole, p1) < d2:
                p3 = pole
                closest_from_pair = p1
                d2 = self.dist_two_poles(pole, p1)
        #switch p0 and p1 if they were assigned incorrectly
        if closest_from_pair == p0:
            p0 = p1
            p1 = closest_from_pair
        rosprint([p0,p1,p2])
        return [p0,p1,p2]


    def dist_two_poles(self, pole0, pole1):
        d = 0
        z0 = self.dist_to_robot(pole0)
        z1 = self.dist_to_robot(pole1)
        phi = np.arctan(pole0.y / pole0.x) - np.arctan(pole1.y / pole1.x)
        #phi = np.pi/2 - np.arctan(pole0.x/pole0.y) - np.arctan(pole1.x/pole1.y)
        d = np.sqrt(z0*z0 + z1*z1 - 2*z0*z1*np.cos(phi))
        rosprint("d1:{},d2:{},phi:{}".format(z0,z1,phi))
        return d

    def get_map_unit(self, close_poles):
        if close_poles is not None:
            map_unit = 0
            d1 = self.dist_two_poles(close_poles[0],close_poles[1])
            d2 = self.dist_two_poles(close_poles[1], close_poles[2])
            rosprint("The two pole dist:{},{}".format(d1,d2))
            rosprint("Relation of the two poles:{}".format(d1/d2))
            rosprint("Deltas to the closest possible relations:{},{},{},{}".format(d1/d2 - 2/3.0,
                                                                                   d1/d2 - 3 / 5.0,
                                                                                   d1 / d2 - 1 / 2.0,
                                                                                   d1 / d2 - 0.5 / 3.0))
            if d1==0 or d2==0:
                return map_unit
            elif d1/d2 - 2/3.0 < 0.05:
                map_unit = d2*6.6666/5.0
            elif d1/d2 - 3/5.0 < 0.05:
                map_unit = d2 * 4.0/5.0
            elif d1/d2 - 1/2.0 < 0.05:
                map_unit = d2 * 4.0 / 5.0
            elif d1/d2 - 0.5/3.0 < 0.05:
                map_unit = d2/3.0
            return map_unit

    def build_map(self):
        #close_poles = self.get_closest_poles(self.extract_poles())
        close_poles = self.extract_poles()
        map_unit = self.get_map_unit(close_poles)
        if map_unit!=0:
            rosprint("Detected map unit:{}".format(map_unit))
            rosprint("The map dimensions are:{},{}".format(map_unit*3, map_unit*5))
            return True


    #server functions
    def check_preemt(self):
        if self.server.is_preempt_requested():
            #rosprint("Preemtied: build_map")
            self.server.set_preempted()
            return True

    def execute(self, goal):
        #rospy.loginfo("Executing build map!")
        result = BuildMapResult()
        obstacle = False
        detect_stuff = Bool()
        detect_stuff.data = 1
        self.OR_pub.publish(detect_stuff)
        if len(self.extract_poles()) >= 3:
            rosprint("Three poles detected! Trying to build map!")
            if self.build_map() is True:
                self.map_init = True
                rosprint("Built map succesfully!")
        self.feedback.message = "something"
        self.server.publish_feedback(self.feedback)
        if self.map_init is True:
            result.message = "succeeded"
            self.server.set_succeeded(result)
        else:
            result.message = "build_map_failed"
            self.server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node("build_map_server")
    build_map_server = BuildMapServer()

    rospy.spin()
