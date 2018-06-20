#!/usr/bin/env python

import rospy
from player.msg import *
import numpy as np


#from tools import rosprint


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

        if self.current_camera_msg is not None:
            for laser_object in scanned_objs_message.scannedObjList:
                laser_coords = np.array((laser_object.x, laser_object.y))
                for kinect_object in self.current_camera_msg.kinectObjList:
                    kinect_coords = np.array((kinect_object.x, kinect_object.y))
                    distance = np.linalg.norm(laser_coords-kinect_coords)
                    if distance < 0.2:
                        matches.append((kinect_coords, kinect_object.color))
            rospy.loginfo("objects matched at: {}".format(matches))

            matched_objects = DetectedObjs()
            matched_objects.header = self.current_camera_msg.header
            for match in matches:
                message = DetectedObj()
                message.x = match[0][0]
                message.y = match[0][1]
                message.id = match[1]
                matched_objects.detectedObjList.append(message)
            self.detected_obj_pub.publish(matched_objects)



if __name__ == '__main__':
    match_node = MatchNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop_rate.sleep()
