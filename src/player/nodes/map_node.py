#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from player.msg import KinectObj, KinectObjs

import tf


class MapNode:



    def __init__(self):
        rospy.init_node("map_node", anonymous=True)
        rospy.loginfo("Initialised map node!")
        # subscribe to kinect and transformations
        self.sub = rospy.Subscriber("camera_objs", KinectObjs, self.objs_cb)
        self.occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        self.pose = tf.TransformListener()

        # map parameters:
        # resolution in meters
        self.resolution = 0.01
        self.height = 500
        self.width = 300

    def map_node_ready(self):
        try:
            map_node_rdy = rospy.Service('laser_node_rdy', map_node_rdy)
            if not map_node_rdy:
                map_node_rdy = True
            return map_node_rdy
        except rospy.ServiceException, e:
            print "Service failed: %s" % e

    def objs_cb(self, msg):
        # initialise map here, fill with loop
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = rospy.Time.now()

        # fill map_msg with the parameters from launchfile
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = range(self.width * self.height)

        map_msg.info.origin.position.x = - self.width // 2 * self.resolution
        map_msg.info.origin.position.y = - self.height // 2 * self.resolution

        # initialize grid with 0 (free) or -1 (unknown)
        grid = np.ndarray((self.width, self.height), buffer=np.zeros((self.width, self.height), dtype=np.int),
                          dtype=np.int)

        grid.fill(int(0))

        t = self.pose.getLatestCommonTime("robot1/kinect_link", "map")
        location, orientation = self.pose.lookupTransform("map", "robot1/kinect_link", t)

        for i in msg.kinectObjList:
            kinect_object = (i.x, i.y)
            self.set_obstacle(grid, location, orientation, kinect_object)

        for i in range(self.width * self.height):
            map_msg.data[i] = grid.flat[i]

        self.occ_pub.publish(map_msg)

    def set_obstacle(self, grid, location, orientation, obstacle):
        # set the occupied cells when detecting an obstacle
        # grid:				ndarray [width,height]
        # position:			[x y] pose of the robot
        # orientation:      quaternion, orientation of the robot

        off_x = location[0] // self.resolution + self.width // 2
        off_y = location[1] // self.resolution + self.height // 2

        euler = tf.transformations.euler_from_quaternion(orientation)

        if True:

            # rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])
            # rospy.loginfo("Obstacle on the map: x={}, y={}".format(off_x,off_y))
            # set probability of occupancy to 100 and neighbour cells to 50
            grid[int(obstacle[0]), int(obstacle[1])] = int(100)
            if grid[int(obstacle[0] + 1), int(obstacle[1])] < int(1):
                grid[int(obstacle[0] + 1), int(obstacle[1])] = int(50)
            if grid[int(obstacle[0]), int(obstacle[1] + 1)] < int(1):
                grid[int(obstacle[0]), int(obstacle[1] + 1)] = int(50)
            if grid[int(obstacle[0] - 1), int(obstacle[1])] < int(1):
                grid[int(obstacle[0] - 1), int(obstacle[1])] = int(50)
            if grid[int(obstacle[0]), int(obstacle[1] - 1)] < int(1):
                grid[int(obstacle[0]), int(obstacle[1] - 1)] = int(50)

            for n in range(-10, 10):
                grid[int(obstacle[0] + n), int(obstacle[1] + n)] = 100
            """
t = 0.5
i = 1
free_cell = np.dot(rotMatrix,np.array([0, t*i])) + np.array([off_x,off_y])

while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
    grid[int(free_cell[0]), int(free_cell[1])] = int(0)
    free_cell = np.dot(rotMatrix,np.array([0, t*i])) + np.array([off_x,off_y])
    i = i+1
"""


if __name__ == '__main__':

    mapnode = MapNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
