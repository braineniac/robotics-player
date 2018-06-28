#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import tf2_ros
import PyKDL

from player.msg import *
from player import rosprint

class TFNode:
    def __init__(self, queue_size=1000):
        rospy.init_node("tf_node")
        rosprint("Initialised transform node!")

#        self.laser_sub =  rospy.Subscriber("front_laser/scan",LaserScan,
#                               self.laser_sub_cb)
        self.depth_sub = rospy.Subscriber("kinect_objs", KinectObjs, self.depth_sub_cb)

#        self.laser_pub = rospy.Publisher("laser_top_shield", LaserScan,queue_size)
        self.depth_pub = rospy.Publisher("kinect_objs_transformed", KinectObjs, queue_size=1)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        rospy.spin()

    def laser_sub_cb(self, laser_msg):
        """
        This is currently never called. Transformation is missing.
        """
        now = rospy.Time.now()
        top_shield_trans = self.tf_buf.lookup_transform("robot1/front_laser",
                "robot1/top_shield_link", now)
        top_shield_laser_msg = self.laser_msg_trans(laser_msg,top_shield_trans)
        self.laser_pub.publish(top_shield_laser_msg)

    def transform_to_kdl(self,t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x,
                t.transform.rotation.y,t.transform.rotation.z, t.transform.rotation.w),
                PyKDL.Vector(t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z))


    def tf_node_ready(self):
        try:
            tf_node_rdy = rospy.Service('laser_node_rdy', tf_node_rdy)
            if not tf_node_rdy:
                tf_node_rdy = True
            return  tf_node_rdy
        except rospy.ServiceException, e:
            print "Service failed: %s"%e


    def do_transform_cloud(self, point, transform):

        t_kdl = self.transform_to_kdl(transform)
        p_out = t_kdl * PyKDL.Vector(point[0], point[1], point[2])
        return p_out

    def depth_sub_cb(self, kinObjList):
        """
        Currently transforming the depth pointcloud into the front laser frame,
        because we can't convert the LaserScan into an another frame
        """
        if kinObjList:
            now = rospy.Time.now()
            can_trans = self.tf_buf.can_transform("robot1/front_laser",
                    "robot1/kinect_depth_optical_frame", now)
            if can_trans:
                top_shield_trans=self.tf_buf.lookup_transform("robot1/front_laser",
                    "robot1/kinect_depth_optical_frame",now)
                kinect_objs_transformed = KinectObjs()
                kinect_objs_transformed.header = kinObjList.header
                for Object in kinObjList.kinectObjList:
                    rosprint(Object)
                    transformed_object = KinectObj()
                    point = (Object.x, Object.y, Object.z)
                    rosprint(point)
                    transformed_point = self.do_transform_cloud(point, top_shield_trans)
                    rosprint(transformed_point)
                    transformed_object.x = transformed_point[0]
                    transformed_object.y = transformed_point[1]
                    transformed_object.z = transformed_point[2]
                    transformed_object.color = Object.color
                    kinect_objs_transformed.kinectObjList.append(transformed_object)
                self.depth_pub.publish(kinect_objs_transformed)

    def laser_msg_transform(laser_msg=None,trans=None):
        """
        Conversion of the LaserScan into another frame. WIP
        """
        if laser_msg and trans:
            laser_msg_trans = LaserScan()
            laser_ms_trans.header = laser_msg.header
            laser_ms_trans.angle_min = laser_msg.angle_min
            laser_ms_trans.angle_max = laser_msg.angle_max
            laser_ms_trans.angle_increment = laser_msg.angle_increment
            laser_ms_trans.time_increment = laser_msg.time_increment
            laser_ms_trans.scan_time = laser_msg.scan_time
            laser_ms_trans.range_min = laser_msg.range_min
            laser_ms_trans.range_max = laser_msg.range_max
            laser_ms_trans.intensities = laser_msg.intensities
            ranges = []
            for i in range(0,360):
               x_vector = PyKDL.Vector(x,0,0)
            #TODO

if __name__ == '__main__':

    tf_node = TFNode()
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
