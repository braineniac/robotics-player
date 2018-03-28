#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
import message_filters
from geometry_msgs.msg import Twist

from player import Player

class PlayNode:
    def __init__(self, image_window="Camera Input", message_slop=0.1, synchroniser_queuesize=20):
        """
        :param message_slop: Messages with a header.stamp within message_slop
        seconds of each other will be considered to be synchronisable
        """
        rospy.init_node("robot_node")
        rospy.loginfo("Initialised PlayNode")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.player = Player(self.vel_pub)
        self.laser_sub = rospy.Subscriber("front_laser/scan", LaserScan,
                self.player.run)
        self.image_sub = message_filters.Subscriber("front_camera/image_raw",
                Image)
        laserSub = message_filters.Subscriber("front_laser/scan", LaserScan)

        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub,
            laserSub], synchroniser_queuesize,message_slop)
        self.time_sync.registerCallback(self.player.camera.show)


if __name__ == '__main__':

    play_node = PlayNode()

   # play_node.laser_listener()
    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
