#!/usr/bin/env python

import random as rd
import sys
import rospy
from geometry_msgs.msg import Twist
from player.msg import *
from player import rosprint


class RefereeNode:
    def __init__(self):
        rospy.init_node("referee_node",anonymous=True)
        rosprint("Referee_node initialised!")

        self.referee_pub = rospy.Publisher
        self.laser_node_rdy = rospy.Subscriber("laser_node_rdy",Bool)


    def server_rdy(req):

    def node_ready(self, node_status):
        """
        Service which asks every node for status and gives boolean about back.
        """
        rospy.wait_for_service('laser_node_rdy')
        rospy.wait_for_service('camera_node_rdy')
        rospy.wait_for_service('map_node_rdy')
        rospy.wait_for_service('match_node_rdy')
        rospy.wait_for_service('move_node_rdy')
        rospy.wait_for_service('odom_node_rdy')
        rospy.wait_for_service('player_node_rdy')
        rospy.wait_for_service('tf_node_rdy')

        try:
        laser_node_rdy = rospy.ServiceProxy('laser_node_rdy', laser_node_rdy)

        camera_node_rdy = rospy.ServiceProxy('camera_node_rdy', camera_node_rdy)

        map_node_rdy = rospy.ServiceProxy('map_node_rdy',map_node_rdy)

        match_node_rdy = rospy.ServiceProxy('match_node_rdy',match_node_rdy)

        move_node_rdy = rospy.ServiceProxy('move_node_rdy', move_node_rdy)

        odom_node_rdy = rospy.ServiceProxy('odom_node_rdy',odom_node_rdy)

        player_node_rdy = rospy.ServiceProxy('player_node_rdy',player_node_rdy)

        tf_node_rdy = rospy.ServiceProxy('tf_node_rdy',tf_node_rdy)
        if laser_node_rdy & camera_node_rdy & map_node_rdy & match_node_rdy & move_node_rdy & odom_node_rdy & player_node_rdy & tf_node_rdy:
            node_status = True
        return node_status

        except rospy.ServiceException, e:
            print "Nodes failed to call: %s"%e



    def teamready(self, team):
        self.referee_pub.publish(msg)

    def sendcolor(self, team, color):



    def senddimensions(self, team, geometry_msgs)


    if __name__ == '__main__':
        node_status = False
        referee_node = RefereeNode()
        if not node_status:
            node_status = node_ready(node_status)
        else
            teamready()

        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 10 Hz loop
            loop_rate.sleep()