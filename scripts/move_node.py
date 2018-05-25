#!/usr/bin/env python

import rospy
from team3_msgs.msg import *

class MoveNode:

    def __init__(self):
        rospy.init_node("move_node",anonymous=True)
        rosprint("Initialised move node!")

        self.move_sub = rospy.Subscriber("cmd_move", CmdMove, self.move)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)

        rospy.spin()

    def move(self, move_msg):
        """
        Executes move based on CmdMove message.
        """

        timeLeft = move_msg.duration
        while timeLeft > 0:
            if move_msg.direction.lower() == "fwd":
                self.forward(move_msg.speed)
            elif move_msg.direction.lower() == "cw":
                self.turnRight(move_msg.speed)
            elif move_msg.direction.lower() == "ccw":
                self.turnLeft(move_msg.speed)
            else:
                self.stop()
            if timeLeft >= 0.25:
                rospy.sleep(0.25)
                timeLeft -= 0.25
            else:
                rospy.sleep(timeLeft)
                timeLeft = 0

        if move_msg.direction.lower() == "fwd":
            self.forward(move_msg.speed)
        elif move_msg.direction.lower() == "cw":
            self.turnRight(move_msg.speed)
        elif move_msg.direction.lower() == "ccw":
            self.turnLeft(move_msg.speed)
        else:
            self.stop()
        rospy.sleep(move_msg.duration)


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

    move_node = MoveNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
