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

        TODO: feat_betterplaynode, priority 3
            Change the attributes to a dictionaries and add init functions to
            initialise them. That means that the actual rostopic strings should
            be only in the main function, the playnode init functions should be
            generic.
        """
        rospy.init_node("robot_node",anonymous=True)
        rospy.loginfo("Initialised PlayNode")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.player = Player(self.vel_pub)
        self.image_sub = message_filters.Subscriber("front_camera/image_raw",
                Image)
        self.laser_sub = message_filters.Subscriber("front_laser/scan", LaserScan)
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub,
            self.laser_sub], synchroniser_queuesize,message_slop)
        self.time_sync.registerCallback(self.player.camera.show)
        self.laser_sub.registerCallback(self.player.run)

if __name__ == '__main__':

    play_node = PlayNode()

    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
