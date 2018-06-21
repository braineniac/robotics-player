#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,Quaternion

from player.msg import ScannedObjs, ScannedObj, KinectObj,KinectObjs, DeltaPose
from player import rosprint

class OdomNode:

    def __init__(self):
        rospy.init_node("odom_node")
        rosprint("Initialised odom node!")

        self.delta_pose_pub = rospy.Publisher("pose_delta", DeltaPose, queue_size=1000)
        self.scanned_sub = rospy.Subscriber("kinect_objs", KinectObjs, self.update_pose)

        self.last_time_stamp = rospy.Time.now()
        self.last_kinect_msg = None
        self.last_v = None

        self.delta_x = 0
        self.delta_y = 0
        self.delta_t = 0
        self.delta_phi = 0

        rospy.spin()

    def odom_node_ready(self):
        try:
            odom_node_rdy = rospy.Service('laser_node_rdy', odom_node_rdy)
            if not odom_node_rdy:
                odom_node_rdy = True
            return  odom_node_rdy
        except rospy.ServiceException, e:
            print "Service failed: %s"%e


    def update_pose(self, kinect_msg):
        """
        Calculates changes of position and rotation of the laser and publishes it.
        """

        if self.last_kinect_msg is None:
            self.last_kinect_msg = kinect_msg
            return
        time_now = kinect_msg.header.stamp.secs + kinect_msg.header.stamp.nsecs/1000000000.0
        time_last = self.last_kinect_msg.header.stamp.secs + self.last_kinect_msg.header.stamp.nsecs/1000000000.0
        dt = time_now - time_last
        if dt < 0.2:
            return
        middle_obj = self.find_middle(self.last_kinect_msg.kinectObjList)
        if(middle_obj == None):
            return
        closest_obj = self.find_closest(middle_obj, kinect_msg.kinectObjList)
        if(closest_obj == None):
            return

        #delta_phi = middle_obj.angle - closest_obj.angle
        if closest_obj.color != "G" or closest_obj.color != "G":
            return
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
        #phi_d = middle_obj.angle - closest_obj.angle
        #phi = np.deg2rad(phi_d)
        dist = np.sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * np.cos(phi))
        delta_x = np.sin(phi) * d1
        delta_y = np.cos(phi) * d1
        #delta_x = np.sin(closest_obj.angle) * closest_obj.dist - np.sin(middle_obj.angle) * middle_obj.dist
        #delta_y = np.cos(closest_obj.angle) * closest_obj.dist - np.cos(middle_obj.angle) * middle_obj.dist
        v_phi = phi_d / dt
        v_delta_x = delta_x / dt
        v_delta_y = delta_y / dt
        #rosprint("Angular velocity:{}".format(v_phi))

        #rosprint("Estimated phi from last velocity:{}".format(v_phi_last * dt))
        #rosprint("dist:{},delta_x:{},delta_y:{},phi:{}".format(dist,delta_x,delta_y,phi_d))
        if phi_d > 20 and self.last_v is not None:
            _x,_y,v_phi_last = self.last_v
            self.delta_phi = self.delta_phi + v_phi_last * dt
        elif v_phi > 20:
            self.delta_phi = self.delta_phi + phi_d
        else:
            return
        #    self.phi = self.phi + phi_d
        #rosprint("Phi from est. vel:{}".format(self.phi_v))
        #rosprint("Phi:{}".format(self.delta_phi))
        self.delta_x = delta_x
        self.delta_y = delta_y
        self.delta_phi = phi
        self.delta_t = dt
        self.last_kinect_msg = kinect_msg
        self.last_v = (v_delta_x,v_delta_y,v_phi)
        self.pub_delta_pose()


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

    def pub_delta_pose(self):
        odom_msg = DeltaPose()
        odom_msg.delta_phi = self.delta_phi
        odom_msg.delta_x = self.delta_x
        odom_msg.delta_y = self.delta_y
        odom_msg.delta_time = self.delta_t
        self.delta_pose_pub.publish(odom_msg)

if __name__ == '__main__':

    odom_node = OdomNode()
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        loop_rate.sleep()
