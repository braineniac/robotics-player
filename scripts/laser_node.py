#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from tools import rosprint

from team3_msgs.msg import ScannedObjs,ScannedObj

class LaserNode:
    def __init__(self):

        #inialising the node and publishers/subsribers
        rospy.init_node("laser_node",anonymous=True)
        rospy.loginfo("initialised laser node!")
        self.laser_sub = rospy.Subscriber("front_laser/scan",LaserScan,self.show)
        self.scanned_obj_pub = rospy.Publisher("scanned_objs", ScannedObjs, queue_size=10)
        #parametrs
        self.data = []
        self.average_data = []

        #keeps node from exiting
        rospy.spin()

    def show(self, laser_msg=None):
        pass

    def slice(self,p=None):
        """
        TODO: feat_tools, priority 3
            Move slice and getDataSize to a new module called tools.py.
        """
        if p is not None:
            if self.getDataSize(self.average_data)<p:
                raise ValueError("p is bigger than raw_laser_data size!\n")
                exit(-1)
            return self.average_data[p:-p]
        else:
            raise ValueError("Passed wrong argument to function!\n")
            exit(-1)

    def checkReliability(slef, laser_msg=None, threshold=0):
        """
        Retursn only reliable ranges. Sets all unreliable values to nan.
        """
        if laser_msg is not None:
            laser_ranges = list(laser_msg.ranges)
            for i in range(len(laser_ranges)):
                if all([v==0 for v in laser_msg.intensities]):
                    return laser_ranges
                if laser_msg.intensities[i] <= threshold:
                    laser_ranges[i] = np.nan
            return laser_ranges

    def save(self,laser_ranges=None):
        """
        Saves the read data from the scan into an attribute.
        """
        if laser_ranges is not None:
            if(len(self.data) >= 5):
                return True
            else:
                self.data.append(laser_ranges)
                return False

    def average(self, laser_ranges_lists):
        """
        Returns list of average values of passed list of lists. If index in one of the passed lists is NaN, same index in returned list in NaN.
        """
        return np.mean(np.array(laser_ranges_lists), axis=0)

    def interpolateMissing(self, laser_ranges, max_interpol_dist=2):
        """
        Returns list with interpolated values for NaNs in passed list.
        """
        for i,val in enumerate(laser_ranges):
            if np.isnan(val):
                left = np.inf
                right = np.inf
                for offset in range(1,max_interpol_dist):
                    try:
                        if(not np.isnan(laser_ranges[i+offset])):
                            left = laser_ranges[i+offset]
                            break
                    except IndexError:
                        pass
                for offset in range(1,max_interpol_dist):
                    try:
                        if(not np.isnan(laser_ranges[i-offset])):
                            right = laser_ranges[i-offset]
                            break
                    except IndexError:
                        pass
                val = np.average([left,right])

        return laser_ranges

    def processData(self, laser_ranges_lists):
        """
        Returns processed list of data. Averages passed lists, interpolates missing values.
        """
        return self.interpolateMissing(self.average(laser_ranges_lists))

    def getDataSize(self, laser_data):
        """
        TODO: see slice
        """
        if laser_data is not None:
            sum =0
            for data in laser_data:
                sum = sum+1
            return sum
        else:
            raise ValueError("Couldn't compute laser data size!\n")

    def obstacle_position(self,threshhold=0):
        """
        TODO: feat_enum_pos, priority 5
            Implement an enum to return where the object is, instead of the
            numbers. Change this also in players avoid_obstacle.
        """
        if self.average_data == []:
            return []
        elif threshhold > 0:
            middle = int(self.getDataSize(self.average_data)/2)
            posphi = 0
            detected_objs = []
            for data in self.average_data:
                try:
                    if data < threshhold:
                        detected_obj = (data, middle - posphi)
                        detected_objs.append(detected_obj)
                except:
                    pass
                posphi += 1
            return detected_objs

        else:
            raise ValueError("The threshhold can't be zero!\n");
            exit(-1)

if __name__ == '__main__':

    laser = LaserNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
