#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import message_filters

import cv2
from cv_bridge import CvBridge, CvBridgeError


class PlayNode:
    def __init__(self, image_window="Camera Input", message_slop=0.1, synchroniser_queuesize=20):
        """
        :param message_slop: Messages with a header.stamp within message_slop
        seconds of each other will be considered to be synchronisable
        """
        rospy.init_node("robot_node")
        rospy.loginfo("Initialised PlayNode")

        self.bridge = CvBridge()
        self.image_window = image_window

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.image_sub = message_filters.Subscriber("front_camera/image_raw", Image)
        self.laser_sub = message_filters.Subscriber("front_laser/scan", LaserScan)

        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.laser_sub],
                                                                     synchroniser_queuesize,
                                                                     message_slop)
        self.time_sync.registerCallback(self.perception_cb)

    def perception_cb(self, img_msg, laser_msg):
        rospy.loginfo("Received new image ({}) and scan ({})".
                format(img_msg.header.stamp, laser_msg.header.stamp))

        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        image = cv2.flip(image, -1)
        cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
        while True:
            print(laser_msg.ranges)
            laser_data = self.laser_slice(laser_msg.ranges,10)
            print(laser_data)
            print(self.obstacle_position(laser_data,5.5))
            exit(0)
            #self.set_velocities(1.0, 0.5)



    def obstacle_position(self,laser_data=None, threshhold=None):
        if laser_data is not None and threshhold is not None:
            middle = int(self.get_laser_datasize(laser_data)/2)
            print(middle)
            laser_middle_data =[laser_data[middle-1],laser_data[middle],laser_data[middle+1]]
            laser_left_data= laser_data[:middle-1]
            laser_right_data = laser_data[middle+1:]

            for data in laser_middle_data:
                try:
                    if data<threshhold:
                        return 0
                except:
                    pass
            for data in laser_right_data:
                try:
                    if data<threshhold:
                        return 1
                except:
                    pass

            for data in laser_left_data:
                try:
                    if data<threshhold:
                        return -1
                except:
                    pass
            return -5

        else:
            raise ValueError("Didn't pass an argument!\n");
            exit(-1)


    def laser_slice(self,raw_laser_data=None,p=None):
        if raw_laser_data is not None and p is not None:
            if self.get_laser_datasize(raw_laser_data)<p:
                raise ValueError("p is bigger than raw_laser_data size!\n")
                exit(-1)
            return raw_laser_data[p:-p]
        else:
            raise ValueError("Passed wrong argument to function!\n")
            exit(-1)

    def set_velocities(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)

    def get_laser_datasize(self,laser_data):
        """
        There is no __len__ implementation of the PlayNode object
        """
        if laser_data is not None:
            sum =0
            for data in laser_data:
                sum = sum+1
            return sum
        else:
            raise ValueError("Couldn't compute laser data size!'n")
            exit(-1)


if __name__ == '__main__':

    play_node = PlayNode()

    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # 10 Hz loop

        loop_rate.sleep()
