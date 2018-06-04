#!/usr/bin/env python

import rospy
from team3_msgs.msg import ScannedObjs, ScannedObj, Odom, KinectObj,KinectObjs
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

        self.scanned_sub = rospy.Subscriber("kinect_objs", KinectObjs, self.update_laser_pose)

        self.last_time_stamp = rospy.Time.now()
        self.last_kinect_msg = None

        self.laser_trans = [0,0,0]
        self.laser_rot_euler = [0,0,0]

    def scanned_cb(self, scanned_msg):
        if self.last_scanned_msg is None:
            self.last_scanned_msg = scanned_msg
            return
        laser_time_now = rospy.Time.now()
        laser_time_last = self.last_time_stamp
        dt = laser_time_now - laser_time_last
        current_peaks = []
        last_peaks = []
        current_ranges = []
        phi,delta_x,delta_y = None,None,None
        for elem_current in scanned_msg.scannedObjList:
            dist_current = elem_current.dist
            angle_current = elem_current.angle
            for elem_last in self.last_scanned_msg.scannedObjList:
                dist_last = elem_last.dist
                angle_last = elem_last.angle
                if angle_last-5 > angle_current and angle_last+5 > angle_current:
                    if dist_current - dist_last < 0.5:
                        phi = angle_current - angle_last
                        delta_x = np.cos(angle_current) * dist_current - np.cos(angle_last) * dist_last
                        delta_y = np.sin(angle_current) * dist_current - np.cos(angle_last) * dist_last
                        #vx = delta_x / dt
                        #vy = delta_y / dt
                        break
            break
        if phi is not None:
            rosprint("phi={},deltax={},deltay={}".format(phi, delta_x,delta_y))
            self.last_scanned_msg = scanned_msg
    #     self.publish(phi, delta_x, delta_y, vx, vy)

    def update_laser_pose(self, kinect_msg):
        """
        Updates position and rotation of the laser and broadcasts it.
        """
        if self.last_kinect_msg is None:
            self.last_kinect_msg = kinect_msg
            return
        time_now_secs = kinect_msg.header.stamp.secs
        time_now_nsecs = kinect_msg.header.stamp.nsecs
        time_last_nsecs = self.last_kinect_msg.header.stamp.nsecs
        time_last_secs = self.last_kinect_msg.header.stamp.nsecs
        time_diff_secs = time_now_secs - time_last_secs
        time_diff_nsecs =  time_now_nsecs - time_last_nsecs
        if time_diff_secs == 0 and time_diff_nsecs < 200000000:
            rosprint("Odom skipped!")
            rosprint("Time diff: secs:{},nsecs:{}".format(time_diff_secs,time_diff_nsecs))
            return
        rosprint("Running update!")
        middle_obj = self.find_middle(self.last_kinect_msg.kinectObjList)
        if(middle_obj == None):
            return
        closest_obj = self.find_closest(middle_obj, kinect_msg.kinectObjList)
        if(closest_obj == None):
            return
        #delta_phi = middle_obj.angle - closest_obj.angle

        x2 = closest_obj.x
        y2 = closest_obj.y
        x1 = middle_obj.x
        y1 = middle_obj.y
        d1 = np.sqrt(x1 * x1 + y1 * y1)
        d2 = np.sqrt(x2 * x2 + y2 * y2)
        x1n = x1 /d1
        y1n = y1 /d1
        x2n = x2 /d2
        y2n = y2 /d2
        #rosprint("x1:{},x2:{},y1:{},y2:{}".format(x1n,x2n,y1n,y2n))
        phi = np.arccos(x1n*x2n+y1n*y2n)
        phi_d = np.rad2deg(phi)
        #rosprint("d1:{},d2:{}".format(d1,d2))
      #  phi_d = middle_obj.angle - closest_obj.angle
       # phi = np.deg2rad(phi_d)
        dist = np.sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * np.cos(phi))
        delta_x = np.sin(phi) * d1
        delta_y = np.cos(phi) * d1
       # delta_x = np.sin(closest_obj.angle) * closest_obj.dist - np.sin(middle_obj.angle) * middle_obj.dist
       # delta_y = np.cos(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist
        rosprint("dist:{},delta_x:{},delta_y:{},phi:{}".format(dist,delta_x,delta_y,phi_d))
        self.laser_trans[0] = delta_x
        self.laser_trans[1] = delta_y
        self.laser_rot_euler[2] = phi
        self.last_kinect_msg = kinect_msg

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
            #delta_x = np.cos(obj.angle) * obj.dist - np.cos(ref_obj.angle) * ref_obj.dist
            #delta_y = np.sin(obj.angle) * obj.dist - np.cos(ref_obj.angle) * ref_obj.dist
            #dist = np.sqrt(delta_x ** 2 + delta_y ** 2)
            x2 = obj.x
            y2 = obj.y
            x1 = ref_obj.x
            y1 = ref_obj.y
            d1 = np.sqrt(x1 * x1 + y1 * y1)
            d2 = np.sqrt(x2 * x2 + y2 * y2)
            x1n = x1 / d1
            y1n = y1 /d1
            x2n = x2 /d2
            y2n = y2 /d2
            phi = np.arccos(x1n*x2n+y1n*y2n)
            phi = np.degrees(phi)
            dist = np.sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * np.cos(phi))
            #rosprint("Dist:{}".format(dist))
            #rosprint("middle_dist={},last_dist={}".format(ref_obj.dist,obj.dist))
            if(dist < smallest_dist):
                smallest_dist = dist
                closest_obj = obj
            #rosprint("Running: ref={},last={}".format(ref_obj.angle,obj.angle))
        #rosprint("middle={},closest={}".format(ref_obj,closest_obj))
        return closest_obj


    def find_middle(self,objList):
        """
        Finds middle object from passed list and returns it.
        """
        middle_obj = ScannedObj()
        middle_obj_angle = np.inf
        if(objList == []):
            return None
        for obj in objList:
            angle = np.arctan(obj.y/obj.x)
        #    rosprint(angle)
            if abs(angle) < abs(middle_obj_angle):
                middle_obj = obj
                middle_obj_angle = angle
        if middle_obj_angle == np.inf:
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
   #     rosprint("trans={}\n,rot={}".format(odom_node.laser_trans,odom_node.laser_rot_euler))

        odom_node.broadcast_laser_pose(odom_node.laser_trans, odom_node.laser_rot_euler, "robot1/base_link")

        loop_rate.sleep()
