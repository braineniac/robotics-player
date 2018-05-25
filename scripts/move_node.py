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

    def avoid_obstacle(self,distance=None):
        """
        TODO: Needs a bit of smaching up!
        """
        if distance > 0:
            detected_obj = self.laser.obstacle_position(distance)
            phi_view = 30
            for data in detected_obj:
                range_obj, phi_obj = data
                if abs(phi_obj) < phi_view:
                    if phi_obj > 0:
                        rosprint("Avoiding obstacle, turning right!")
                        self.turnRight(1)
                    else:
                        rosprint("Avoiding obstacle, turning left!")
                        self.turnLeft(1)

            rosprint("No obstacle ahead! Moving randomly!")
            if rd.random() < 0.66:
                self.forward(0.1)
            elif rd.random() < 0.81:
                self.turnRight(0.5)
            else:
                self.turnLeft(0.5)
        else:
            raise ValueError("Obstacle distance can't be negative!\n")

if __name__ == '__main__':

    move_node = MoveNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
