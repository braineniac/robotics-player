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
