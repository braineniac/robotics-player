#!/usr/bin/env python

import random as rd
import rospy
from geometry_msgs.msg import Twist
from team3_msgs.msg import DetectedObjs,DetectedObj,ScannedObj,ScannedObj,KinectObj,KinectObj
from tools import rosprint

class PlayerNode:
    def __init__(self):
        #inialising the node and publishers/subsribers
        rospy.init_node("player_node",anonymous=True)
        rospy.loginfo("Player node initialised.")

        self.detected_objs_sub = rospy.Subscriber("detected_objs", DetectedObjs, self.run)

        self.move_pub = rospy.Publisher("cmd_move", CmdMove, queue_size=1000)
        #keeps node from exiting
        rospy.spin()

    def run(self, detected_objs_msg=None):
        sel.move("fwd",10,5)

    def move(self, direction, duration, speed):
        """
        Sends CmdMove message to cmd_move topic. Direction: "fwd" = forward, "cw" = clockwise, "ccw" = counterclockwise. Duration in seconds. Speed in m/s.
        """
        msg = CmdMove()
        msg.direction = direction
        msg.duration = duration
        msg.speed = speed
        self.move_pub.publish(msg)

if __name__ == '__main__':

    player = PlayerNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
