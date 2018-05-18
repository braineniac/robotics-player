#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import tf2_ros
import tf2_sensor_msgs


from tools import rosprint

class TFNode:
    def __init__(self, queue_size=10):
        rospy.init_node("tf_node")
        rosprint("Initialised transform node!")

        self.laser_sub =  rospy.Subscriber("front_laser/scan",LaserScan,self.laser_sub_cb)
        self.rgb_sub = rospy.Subscriber("kinect/rgb/image_raw", Image,self.rgb_sub_cb)
        self.depth_sub = rospy.Subscriber("kinect/depth/points", PointCloud2, self.depth_sub_cb)

        self.laser_pub = rospy.Publisher("laser_top_shield", LaserScan,queue_size)
        self.rgb_pub = rospy.Publisher("camera_rgb_top_shield", Image,queue_size)
        self.depth_pub = rospy.Publisher("camera_depth_top_shield", PointCloud2, queue_size)

        self.tf_buf = tf2_ros.Buffer()

        rospy.spin()

    def laser_sub_cb(self, laser_msg):
        now = rospy.Time.now()
        top_shield_trans = self.tf_buf.lookup_transform("robot1/front_laser",
                "robot1/top_shield_link", now)
        top_shield_laser_msg = self.tf_buf.transform(laser_msg,"robot1/top_shield_link")
        self.laser_pub(top_shield_laser_msg)

    def rgb_sub(self, img_msg):
        top_shield_img_msg = self.tf_buf.transform(img_msg,"robot1/top_shield_link")
        self.rgb_pub(img_msg)

    def depth_pub(self, point_cloud):
        top_shield_trans = self.tf_buf.lookup_transform("robot1/kinect_depth_optical_frame",
                "robot1/top_shield_link")
        top_shield_pt_msg = tf2_sensor_msgs.do_transform_cloud(point_cloud,
                top_shield_trans)
        self.depth_pub(top_shield_pt_msg)


if __name__ == '__main__':

    tf_node = TFNode()
    loop_rate = rospy.Rate(10)

    rosprint("Starting loop")

    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
