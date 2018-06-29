#!/usr/bin/env python

import rospy
from player.msg import *
import numpy as np

from player import rosprint


class MatchNode:

    def __init__(self):

        rospy.init_node("player_node", anonymous=True)
        rospy.loginfo("Initialised match node!")
        self.scanned_obj_sub = rospy.Subscriber("scanned_objs", ScannedObjs, self.match)
        self.kinect_objs_sub = rospy.Subscriber("kinect_objs_transformed", KinectObjs, self.camera_save)
        self.detected_obj_pub = rospy.Publisher("detected_objs", DetectedObjs, queue_size=1)
        self.sim_env = rospy.get_param('sim_env')

        self.own_goal_det = False
        self.other_goal_det = False
        self.current_camera_msg = None
        rospy.spin()

    def camera_save(self, camera_msg):
        """
        Just saves the camera msg, so the freshest one is used during the laser
        callback(the camera is 3 times faster than the laser).
        """
        if camera_msg:
            self.current_camera_msg = camera_msg


    def match_node_ready(self):
        try:
            match_node_rdy = rospy.Service('laser_node_rdy', match_node_rdy)
            if not match_node_rdy:
                match_node_rdy = True
            return  match_node_rdy
        except rospy.ServiceException, e:
            print "Service failed: %s"%e


    def match(self, scanned_objs_message):

        """
        TODO: make this a lot better after the fixing the laser and camera
        """
        matches = []
        camera_msg = self.current_camera_msg
        if camera_msg is not None and self.sim_env is False:
            #rospy.loginfo(camera_msg.kinectObjList)
            for laser_object in scanned_objs_message.scannedObjList:
                laser_coords = np.array((laser_object.x, laser_object.y))
                for kinect_object in camera_msg.kinectObjList:
                    match_obj = DetectedObj()
                    kinect_coords = np.array((kinect_object.x, kinect_object.y))
                    distance = np.linalg.norm(laser_coords-kinect_coords)
                    match_obj.x = kinect_object.x
                    match_obj.y = kinect_object.y
                    if kinect_object.color == "G":
                        #rosprint(distance)
                        if distance < 0.2:
                            match_obj.id = "pole"
                            if match_obj not in matches:
                                matches.append(match_obj)
                    if kinect_object.z < -0.25 and kinect_object.color != "G":
                        match_obj.id = "{}_goal".format(kinect_object.color)
                        if match_obj not in matches:
                            d = np.sqrt(kinect_object.x * kinect_object.x + kinect_object.y * kinect_object.y)
                            if self.own_goal_det is False:
                                rosprint("Detected our own goal, color:{}".format(kinect_object.color))
                                self.own_goal_det = True
                            if d > 2.0 and self.other_goal_det is False:
                                rosprint("Detected other teams goal, color:{}".format(kinect_object.color))
                                self.other_goal_det = True
                            matches.append(match_obj)
                    elif kinect_object.color != "G":
                        match_obj.id = "{}_puck".format(kinect_object.color)
                        if match_obj not in matches:
                            matches.append(match_obj)

            self.current_camera_msg = None
            #rosprint("================================================================")
            #rospy.loginfo("objects matched at: {}".format(matches))
            #rosprint("================================================================")
            self.publish_matches(matches, camera_msg)

    def publish_matches(self, matches, camera_msg):
        if matches:
            matched_objects = DetectedObjs()
            matched_objects.header = camera_msg.header
            for match in matches:
                matched_objects.detectedObjList.append(match)
            self.detected_obj_pub.publish(matched_objects)

if __name__ == '__main__':
    match_node = MatchNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop_rate.sleep()
