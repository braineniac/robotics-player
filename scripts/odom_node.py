#!/usr/bin/env python

import rospy
from team3_msgs.msg import ScannedObjs, ScannedObj, Odom
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

        self.scanned_sub = rospy.Subscriber("scanned_objs", ScannedObjs, self.update_laser_pose)

        self.last_time_stamp = rospy.Time.now()
        self.last_scanned_msg = ScannedObjs()

        self.laser_trans = [0,0,0]
        self.laser_rot_euler = [0,0,0]

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

    def update_laser_pose(self, scanned_msg):
        """
        Updates position and rotation of the laser and broadcasts it.
        """
        middle_obj = self.find_middle(self.last_scanned_msg.scannedObjList)
        if(middle_obj == None):
            return

        closest_obj = self.find_closest(middle_obj, scanned_msg.scannedObjList)
        if(closest_obj == None):
            return

        delta_phi = middle_obj.angle - closest_obj.angle
        delta_x = np.cos(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist
        delta_y = np.sin(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist

        self.laser_trans[0] += delta_x
        self.laser_trans[1] += delta_y
        self.laser_rot_euler[2] += delta_phi

        self.last_scanned_msg = scanned_msg

    def broadcast_laser_pose(self, trans, rot_euler, laser_name):
        """
        Broadcasts position and rotation of the laser.
        """
        br = tf.TransformBroadcaster()
        br.sendTransform(trans, tf.transformations.quaternion_from_euler(rot_euler[0],rot_euler[1],rot_euler[2]), rospy.Time.now(), laser_name, "map")

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

    def publish(self, phi, delta_x, delta_y, vx,vy):
        odom_msg = Odom()
        odom_msg.phi = phi
        odom_msg.delta_x = delta_x
        odom_msg.delta_y = delta_y
        odom_msg.vx = vx
        odom_msg.vy = vy
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':

    odom_node = OdomNode()
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        odom_node.broadcast_laser_pose(odom_node.laser_trans, odom_node.laser_rot_euler, "robot1/odom")

        loop_rate.sleep()
