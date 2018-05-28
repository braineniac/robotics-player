#!/usr/bin/env python

import rospy
from  sensor_msgs.msg import JointState
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
        self.odom_pub = rospy.Publisher("odom_node", Odometry,queue_size=1000)
        self.broadcaster = tf2_ros.TransformBroadcaster()


if __name__ == '__main__':

    odom_node = OdomNode()
    loop_rate = rospy.Rate(10)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    #initial position
    x = 0.0
    y = 0.0
    th= 0

    vx = 0
    vy = 0
    vth = 0

    #message declarations
    odom_trans = TransformStamped()
    header_frame = odom_trans.header.frame_id = "odom_custom"
    child_frame = odom_trans.child_frame_id = "base_link"


    while not rospy.is_shutdown():
       current_time = rospy.Time.now()
       dt = (current_time - last_time).to_sec()
       delta_x = (vx * np.cos(th) - vy * np.sin(th)) * dt
       delta_y = (vx * np.sin(th) + vy * np.cos(th)) * dt
       delta_th = vth * dt

       x += delta_x
       y += delta_y
       th += delta_th

       odom_quat = Quaternion()
       odom_quat =tf.transformations.quaternion_from_euler(0,0,th)

       # update transform
       odom_trans.header.stamp = current_time
       odom_trans.transform.translation.x = x
       odom_trans.transform.translation.y = y
       odom_trans.transform.translation.z = 0.0
       odom_trans.transform.rotation = tf.transformations.quaternion_from_euler(0,0,th)

       #filling the odometry
       odom = Odometry()
       odom.header.stamp = current_time
       odom.header.frame_id = "odom_custom"
       odom.child_frame_id = "base_link"

       #position
       odom.pose.pose.position.x = x
       odom.pose.pose.position.y = y
       odom.pose.pose.position.z = 0.0
       odom.pose.pose.orientation = odom_quat

       #velocity
       odom.twist.twist.linear.x = vx
       odom.twist.twist.linear.y = vy
       odom.twist.twist.linear.z = 0.0
       odom.twist.twist.angular.x = 0.0
       odom.twist.twist.angular.y = 0.0
       odom.twist.twist.angular.z = vth

       last_time = current_time

       odom_node.broadcaster.sendTransform(odom_trans)
       odom_node.odom_pub.publish(odom)

       loop_rate.sleep()
