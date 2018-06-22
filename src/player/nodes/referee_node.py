#!/usr/bin/env python

import random as rd
import sys
import rospy
from player.msg import *
from player import rosprint
from player.msg import waitForTeams, gameControl
from player.srv import *

from src import player


class RefereeNode:
    def __init__(self):
        self.referee_pub_status = rospy.Publisher("waitForTeams", waitForTeams, queue_size=10)
        self.referee_pub_control = rospy.Publisher("gameControl", gameControl, queue_size=10)
        rospy.init_node("referee_node",anonymous=True)
        rosprint("Referee_node initialised!")

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
        laser_status = player.srv.laser_node_rdy
        camera_status = player.srv.camera_node_rdy
        map_status = player.srv.map_node_rdy
        match_status = player.srv.match_node_rdy
        move_status = player.srv.move_node_rdy
        odom_status = player.srv.odom_node_rdy
        player_status = player.srv.player_node_rdy
        tf_status = player.srv.tf_node_rdy


        try:
            laser_node_rdy = rospy.ServiceProxy('laser_node_rdy', laser_status)

            camera_node_rdy = rospy.ServiceProxy('camera_node_rdy', camera_status)

            map_node_rdy = rospy.ServiceProxy('map_node_rdy',map_status)

            match_node_rdy = rospy.ServiceProxy('match_node_rdy',match_status)

            move_node_rdy = rospy.ServiceProxy('move_node_rdy', move_status)

            odom_node_rdy = rospy.ServiceProxy('odom_node_rdy',odom_status)

            player_node_rdy = rospy.ServiceProxy('player_node_rdy',player_status)

            tf_node_rdy = rospy.ServiceProxy('tf_node_rdy',tf_status)
            if laser_node_rdy and camera_node_rdy and map_node_rdy and match_node_rdy and move_node_rdy and odom_node_rdy and player_node_rdy and tf_node_rdy:
                node_status = True
            return node_status

        except rospy.ServiceException, e:
            print "Nodes failed to call: %s"%e

    def team_status(self, team_status):
        if team_status.team == 'A':
            team_status.ok = True
        return team_status

    def teamready(self):
        rospy.wait_for_service('TeamReady')
        teamready = rospy.Service('TeamReady', player.srv.TeamReady, referee_node.team_status)
        rospy.spin()

    def determine_team_color(self, team_color):
        if team_color.team == 'A' and team_color.color == 'blue':
            team_color.ok = True
        else:
            team_color.ok = False
            team_color.correctcolor = 'yellow'
        return team_color


    def sendcolor(self):
        rospy.wait_for_service('SendColor')
        team_color = rospy.Service('SendColor', player.srv.SendColor, referee_node.determine_team_color)
        rospy.spin()



#    def senddimensions(self, team, geometry_msgs):


if __name__ == '__main__':
    referee_node = RefereeNode()
    node_status = False
    if not node_status:
        node_status = referee_node.node_ready(node_status)
    else:
        status = waitForTeams()
        referee_node.referee_pub_status.publish(status)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()