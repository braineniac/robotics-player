#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from team3_msgs.msg import KinectObj, KinectObjs

import tf

class MapNode:
    def __init__(self):
        rospy.init_node("map_node",anonymous=True)
        rospy.loginfo("Initialised map node!")
        # subscribe to kinect and transformations
        self.sub = rospy.Subscriber("robot1/camera_objs", KinectObjs, self.objs_cb)
        occ_pub = rospy.Publisher("/car/map", OccupancyGrid, queue_size=10)

        pose = tf.TransformListener()

        # map parameters:
        self.resolution = 0.1
        self.height = 500
        self.width = 300



    def objs_cb(self, msg)
        #initialise map here, fill with loop
		map_msg = OccupancyGrid()
		map_msg.header.frame_id = 'map'

		# fill map_msg with the parameters from launchfile
		map_msg.info.resolution = resolution
		map_msg.info.width = self.width
		map_msg.info.height = self.height
		map_msg.data = range(width * height)

		# initialize grid with -1 (unknown)
		grid = numpy.ndarray((width, height), buffer=numpy.zeros((width, height), dtype=numpy.int),
							 dtype=numpy.int)

		grid.fill(int(0))

		t = pose.getLatestCommonTime("/car_base_link", "/world")
		location, orientation = pose.lookupTransform("/world", "/base_link", t)

		for i in msg:
			object = (msg.x, msg.y)
			set_obstacle(grid, location, orientation, object)






def set_obstacle(grid, location, orientation, obstacle):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car

	off_x = location[1] // self.resolution + self.width  // 2
	off_y = location[0] // self.resolution + self.height // 2

	euler = tf.transformations.euler_from_quaternion(orientation)

	if True:

		rospy.loginfo("FOUND OBSTACLE AT: x:%f y:%f", obstacle[0], obstacle[1])

		# set probability of occupancy to 100 and neighbour cells to 50
		grid[int(obstacle[0]), int(obstacle[1])] = int(100)
		if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(1):
			grid[int(obstacle[0]+1), int(obstacle[1])]   = int(50)
		if  grid[int(obstacle[0]), 	 int(obstacle[1]+1)] < int(1):
			grid[int(obstacle[0]),   int(obstacle[1]+1)] = int(50)
		if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(1):
			grid[int(obstacle[0]-1), int(obstacle[1])]   = int(50)
		if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(1):
			grid[int(obstacle[0]),   int(obstacle[1]-1)] = int(50)

		t = 0.5
		i = 1
		free_cell = np.dot(rotMatrix,np.array([0, t*i])) + np.array([off_x,off_y])

		while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
			grid[int(free_cell[0]), int(free_cell[1])] = int(0)
			free_cell = np.dot(rotMatrix,np.array([0, t*i])) + np.array([off_x,off_y])
			i = i+1


if __name__ == '__main__':

    mapnode = MapNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
