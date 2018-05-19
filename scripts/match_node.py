#!/usr/bin/env python

import rospy
from team3_msgs.msg import *

from tools import rosprint

class MatchNode:

    def __init__(self):
        rospy.init_node("player_node",anonymous=True)
        rosprint("Initialised match node!")
        self.scanned_obj_sub = rospy.Subscriber("scanned_objs", ScannedObjs, self.match)
        self.kinect_objs_sub = rospy.Subscriber("camera_objs", KinectObjs, self.camera_save)
        self.detected_obj_pub = rospy.Publisher("detected_objs", DetectedObjs, queue_size=1)

        self.current_camera_msg = None
        rospy.spin()

    def match(self,scanned_msg):
        pass

    def camera_save(self, camera_msg):
        if camera_msg:
            self.current_camera_msg = camera_msg

if __name__ == '__main__':
    match_node = MatchNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop_rate.sleep()
