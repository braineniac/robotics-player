#!/usr/bin/env python

import rospy
from team3_msgs.msg import ScannedObjs, ScannedObj, DeltaPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,Quaternion
import tf
import tf2_ros
from tools import rosprint
import numpy as np

class OdomNode:

    def __init__(self):
        rospy.init_node("odom_node")
        rosprint("Initialised odom node!")

        self.scanned_sub = rospy.Subscriber("scanned_objs", ScannedObjs, self.calc_deltas)
        self.pose_pub = rospy.Publisher("pose_deltas", DeltaPose,queue_size=1000)

        self.last_time_stamp = rospy.Time.now()
        self.last_scanned_msg = ScannedObjs()

        rospy.spin()
    # def scanned_cb(self, scanned_msg):
    #     laser_time_now = rospy.Time.now()
    #     laser_time_last = self.last_time_stamp
    #     dt = laser_time_now - laser_time_last
    #     current_peaks = []
    #     last_peaks = []
    #     current_ranges = []
    #     for elem_current in scanned_msg.scannedObjList:
    #         dist_current, angle_current = elem
    #         for elem_last in self.last_scanned_msg:
    #             dist_last,angle_last = elem_last
    #             if angle_last -5>angle_current and angle_last+5 > angle_current:
    #                 if dist_current - dist_last < 0.1:
    #                     phi = angle_current - angle_last
    #                     delta_x = np.cos(angle_current) * dist_current - np.cos(angle_last) * dist_last
    #                     delta_y = np.sin(angle_current) * dist_current - np.cos(angle_last) * dist_last
    #                     vx = delta_x / dt
    #                     vy = delta_y / dt
    #                     break
    #         break
    #     self.publish(phi, delta_x, delta_y, vx, vy)
    #     self.last_scanned_msg = scanned_msg

    def calc_deltas(self, scanned_msg):
        """
        Calculates changes of position and rotation of the laser and publishes it.
        """
        laser_time_now = rospy.Time.now()
        laser_time_last = self.last_time_stamp
        delta_time = laser_time_now - laser_time_last

        middle_obj = self.find_middle(self.last_scanned_msg.scannedObjList)
        if(middle_obj == None):
            return

        closest_obj = self.find_closest(middle_obj, scanned_msg.scannedObjList)
        if(closest_obj == None):
            return

        delta_phi = middle_obj.angle - closest_obj.angle
        delta_x = np.cos(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist
        delta_y = np.sin(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist

        self.publish(delt_phi, delta_x, delta_y, delta_time)

        self.last_scanned_msg = scanned_msg


    def find_closest(self, ref_obj, objList):
        """
        Finds closest object to the reference object and returns it.
        """
        closest_obj = None
        smallest_dist = np.inf
        for obj in objList:
            delta_x = np.cos(obj.angle) * obj.dist - np.cos(ref_obj.angle) * ref_obj.dist
            delta_y = np.sin(obj.angle) * obj.dist - np.cos(ref_obj.angle) * ref_obj.dist
            dist = sqrt(delta_x ** 2 + delta_y ** 2)
            if(dist < smallest_dist):
                smallest_dist = dist
                closest_obj = obj
        return closest_obj

    def find_middle(self,objList):
        """
        Finds middle object from passed list and returns it.
        """
        middle_obj = ScannedObj()
        middle_obj.angle = np.inf
        if(objList == []):
            return None
        for obj in objList:
            if abs(obj.angle) < abs(middle_obj.angle):
                middle_obj = obj
        if middle_obj.angle == np.inf:
            return None
        else:
            return middle_obj

    def publish(self, delta_phi, delta_x, delta_y, delta_time):
        odom_msg = DeltaPose()
        odom_msg.delt_phi = delta_phi
        odom_msg.delta_x = delta_x
        odom_msg.delta_y = delta_y
        odom_msg.delta_time = delta_time
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':

    odom_node = OdomNode()
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        loop_rate.sleep()
