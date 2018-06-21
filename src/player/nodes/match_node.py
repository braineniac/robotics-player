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

        self.current_camera_msg = None
        rospy.spin()

    def camera_save(self, camera_msg):
        """
        Just saves the camera msg, so the freshest one is used during the laser
        callback(the camera is 3 times faster than the laser).
        """
        if camera_msg:
            self.current_camera_msg = camera_msg

    def match(self, scanned_objs_message):
        """
        TODO: make this a lot better after the fixing the laser and camera
        """
        matches = []
        camera_msg = self.current_camera_msg
        if camera_msg is not None:
            rospy.loginfo(camera_msg.kinectObjList)
            for laser_object in scanned_objs_message.scannedObjList:
                laser_coords = np.array((laser_object.x, laser_object.y))
                for kinect_object in camera_msg.kinectObjList:
                    match_obj = DetectedObj()
                    kinect_coords = np.array((kinect_object.x, kinect_object.y))
                    distance = np.linalg.norm(laser_coords-kinect_coords)
                    match_obj.x = kinect_object.x
                    match_obj.y = kinect_object.y
                    if kinect_object.color == "G":
                        if distance < 0.2:
                            match_obj.id = "pole"
                            if match_obj not in matches:
                                matches.append(match_obj)
                                break
                    if kinect_object.z < -0.25:
                        match_obj.id = "{}_goal".format(kinect_object.color)
                        if match_obj not in matches:
                            matches.append(match_obj)
                    elif kinect_object.color != "G":
                        match_obj.id = "{}_puck".format(kinect_object.color)
                        if match_obj not in matches:
                            matches.append((kinect_coords, "{}_puck".format(kinect_object.color)))

            self.current_camera_msg = None
            rosprint("================================================================")
            rospy.loginfo("objects matched at: {}".format(matches))
            rosprint("================================================================")
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
