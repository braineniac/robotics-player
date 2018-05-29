#!/usr/bin/env python

import rospy
from team3_msgs.msg import *

from tools import rosprint

class MatchNode:

    def __init__(self):

        rospy.init_node("player_node",anonymous=True)
        rosprint("Initialised match node!")
        self.scanned_obj_sub = rospy.Subscriber("scanned_objs", ScannedObjs,self.match)
        self.kinect_objs_sub = rospy.Subscriber("camera_objs", KinectObjs,self.camera_save)
        self.detected_obj_pub = rospy.Publisher("detected_objs", DetectedObjs,queue_size=1)

        self.current_camera_msg = None
        rospy.spin()

    def camera_save(self, camera_msg):
        """
        Just saves the camera msg, so the freshest one is used during the laser
        callback(the camera is 3 times faster than the laser).
        """
        if camera_msg:
            self.current_camera_msg = camera_msg


    def match(self,scanned_msgs):
        """
        TODO: make this a lot better after the fixing the laser and camera
        """
        break
        for camera_msg in self.current_camera_msg.kinectObjList:
            x1 = camera_msg.lower
            x2= camera_msg.upper
            area = camera_msg.area
            color = camera_msg.color
            phi1 = 11*(x1-30)/90-55
            phi2 = 11*(x2+30)/90-55
            for scanned_msg in scanned_msgs.scannedObjList:
                laser_phi = scanned_msg.angle
                distance = scanned_msg.dist
                if ((laser_phi>=phi1) and (laser_phi<=phi2)):
                    if color != "G" and area>10000:
                        if color == "Y":
                            rosprint("Detected the yellow goal!")
                        else:
                            rosprint("Detected the blue goal!")
                    elif color == "G":
                        rospy.loginfo("Detected a pole in {} m distance and \
                                {}degrees!".format(distance,laser_phi))
                    elif color == "B":
                        rospy.loginfo("Detected a blue puck in {} m distance \
                                and {} degrees!".format(distance,laser_phi))

if __name__ == '__main__':

    match_node = MatchNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop_rate.sleep()
