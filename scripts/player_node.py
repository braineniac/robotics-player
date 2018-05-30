#!/usr/bin/env python

import random as rd
import rospy
from geometry_msgs.msg import Twist
from team3_msgs.msg import *
from tools import rosprint
import numpy as np

class PlayerNode:
    def __init__(self):
        #inialising the node and publishers/subsribers
        rospy.init_node("player_node",anonymous=True)
        rospy.loginfo("Player node initialised.")

        self.scanned_objs_sub = rospy.Subscriber("scanned_objs", ScannedObjs, self.run)

        self.move_pub = rospy.Publisher("cmd_move", CmdMove, queue_size=1000)
        self.odom_sub = rospy.Subscriber("odom_node", Odom, self.odom_cb)
        self.map_init = False
        self.odom = None
        self.mapped_objs = []
        #keeps node from exiting
        rospy.spin()

    def odom_cb(self, odom_msg):
        self.odom = odom_msg

    def run(self, detected_objs_msg=None):
        self.move("ccw",speed=0.3,duration=10)
      #  if self.map_init is False:
            #self.init_map(detected_objs_msg)

    def check_mapped(self, detectedObjs):
        odom = self.odom
        is_new_obj = True
        for detected_obj in detectedObjs:
            for mapped_obj in self.mapped_objs:
                # add corrections for odom
                x1 = mapped_obj.x
                y1 = mapped_obj.y
                x2 = detected_obj.x
                y2 = detected_obj.y
                d1 = np.sqrt(x1*x1 + y1*y1)
                d2 = np.sqrt(x2*x2 + y2*z2)
                phi = np.arctan(y1/x1) - np.arctan(y2/x2)
                d = sqrt(d1*d1 + d2*d2 -2*d1*d2*np.cos(phi))
                if d < 0.05:
                    is_new_obj = False
            if is_new_obj:
                self.mapped_objs.append(detected_obj)

    def check_for_3_poles(self):
        i = 0
        for mapped_obj in self.mapped_objs:
            if mapped_obj.id == "pole":
                i = i+1
        if i>=3:
            return True
        else:
            return False

    def build_map(self):
        poles_list = []
        for mapped_obj in self.mapped_objs:
            if len(poles_list) >= 3:
                break
            if mapped_obj.id == "pole":
                poles_list.append(mapped_obj)
        x0 = poles_list[0].x
        x1 = poles_list[1].x
        x2 = poles_list[2].x
        y0 = poles_list[0].y
        y1 = poles_list[1].y
        y2 = poles_list[2].y
        z0 = np.sqrt(x0*x0 + y0*y0)
        z1 = np.sqrt(x1*x1 + y1*y1)
        z2 = np.sqrt(x2*x2 + y2*y2)
        phi1 = np.arctan(y0/x0) - np.arctan(y1/x1)
        phi2 = np.arctan(y1/x1) - np.arctan(y2/x2)
        d1 = np.sqrt(z0*z0 + z1*z1 -2*z0*z1*np.cos(phi1))
        d2 = np.sqrt(z1*z1 + z2*z2 -2*z1*z2*np.cos(phi2))
        if d1>d2:
            d1,d2 = d2,d1

        if d1/d2 - 2/3 < 0.01:
            map_unit = d2*6.6666/5
        elif d1/d2 - 3/5 < 0.01:
            map_unit = d2 * 4/5
        elif d1/d2 - 1/2 < 0.01:
            map_unit = d2 * 4 / 5
        elif d1/d2 - 0.5/3 - 0.01:
            map_unit = d2/3


    def init_map(self, detected_objs_msg):
        self.move("ccw", 1, 0.5)
        self.check_mapped(detected_objs_msg)
        if self.check_for_3_poles():
            self.build_map()



    def move(self, direction, duration=0, speed=0):
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

    def avoid_obstacle(self,distance=None):
        """
        Sends CmdMove message to cmd_move topic. Direction: "fwd" = forward, "cw" = clockwise, "ccw" = counterclockwise. Duration in seconds. Speed in m/s.
        """
        if distance > 0:
            detected_obj = self.laser.obstacle_position(distance)
            phi_view = 30
            for data in detected_obj:
                range_obj, phi_obj = data
                if abs(phi_obj) < phi_view:
                    if phi_obj > 0:
                        rosprint("Avoiding obstacle, turning right!")
                        self.move("cw",1,1)
                    else:
                        rosprint("Avoiding obstacle, turning left!")
                        self.move("ccw",1,1)

            rosprint("No obstacle ahead! Moving randomly!")
            if rd.random() < 0.66:
                self.move("fwd",1,0.1)
            elif rd.random() < 0.81:
                self.move("cw",1,0.5)
            else:
                self.move("ccw",1,0.5)
        else:
            raise ValueError("Obstacle distance can't be negative!\n")


if __name__ == '__main__':

    player = PlayerNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
