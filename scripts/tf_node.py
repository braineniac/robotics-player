#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import tf2_ros
#import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msg
from sensor_msgs.point_cloud2 import read_points, create_cloud
import PyKDL
from tools import rosprint

class TFNode:
    def __init__(self, queue_size=1000):
        rospy.init_node("tf_node")
        rosprint("Initialised transform node!")

#        self.laser_sub =  rospy.Subscriber("front_laser/scan",LaserScan,
#                               self.laser_sub_cb)
        self.depth_sub = rospy.Subscriber("kinect/depth/points", PointCloud2, self.depth_sub_cb)

#        self.laser_pub = rospy.Publisher("laser_top_shield", LaserScan,queue_size)
        self.depth_pub = rospy.Publisher("camera_depth", PointCloud2, queue_size=queue_size)

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

    def do_transform_cloud(self,cloud, transform):
            t_kdl = self.transform_to_kdl(transform)
            points_out = []
            for p_in in read_points(cloud):
                p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
                points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
                res = create_cloud(transform.header, cloud.fields, points_out)
            return res

    def depth_sub_cb(self, point_cloud):
        """
        Currently transforming the depth pointcloud into the front laser frame,
        because we can't convert the LaserScan into an another frame
        """
        if point_cloud:
            now = rospy.Time.now()
            can_trans = self.tf_buf.can_transform("robot1/front_laser",
                    "robot1/kinect_depth_frame", now)
            if can_trans:
                top_shield_trans=self.tf_buf.lookup_transform("robot1/front_laser",
                    "robot1/kinect_depth_frame",now)
                top_shield_pt_msg = self.do_transform_cloud(point_cloud,
                    top_shield_trans)
                self.depth_pub.publish(top_shield_pt_msg)

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
