#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import tools
from tools import rosprint

from team3_msgs.msg import ScannedObjs,ScannedObj

class LaserNode:
    def __init__(self):

        #inialising the node and publishers/subsribers
        rospy.init_node("laser_node",anonymous=True)
        rospy.loginfo("Initialised laser node!")
        self.laser_sub = rospy.Subscriber("front_laser/scan",LaserScan,
                self.laser_scan_cb)
        self.scanned_obj_pub = rospy.Publisher("scanned_objs", ScannedObjs,
                queue_size=1000)
        #parametrs
        self.data = []
        self.average_data = []

        #keeps node from exiting
        rospy.spin()

    def laser_scan_cb(self, laser_msg=None):
        if self.save(self.checkReliability(laser_msg)):
             self.average_data = self.processData(self.data)
             self.data = [] #cleans data in memory
        self.obstacle_position()

    def slice(self,p=None):
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
        Returns list of average values of passed list of lists.
        If index in one of the passed lists is NaN, same index in returned
        list in NaN.
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
        Returns processed list of data. Averages passed lists, interpolates
        missing values.
        """
        return self.interpolateMissing(self.average(laser_ranges_lists))

    def obstacle_position(self,threshhold=5):
        if self.average_data == []:
            return []
        elif threshhold > 0:
            middle = int(len(self.average_data)/2)
            posphi = 0
            scanned_angles = ()
            scanned_dists = ()
            for data in self.average_data:
                try:
                    if data < threshhold:
                        scanned_angles += (self, middle - posphi)
                        scanned_dists += (self, data)
                        self.remove_objects(scanned_angles, scanned_dists)

                except:
                    pass
                posphi += 1


        else:
            raise ValueError("The threshhold can't be zero!\n");
            exit(-1)

    def remove_objects(self, scanned_angles, scanned_dists):
        n=-1
        scan_msgs = ScannedObjs()
        dist_threshold = 0.05
        mergable_object_angle = ()
        mergable_object_dist = ()
        merged_objects_angle = ()
        merged_objects_dist = ()
        for scanned_angles[n] in scanned_angles:
            n+=1
            if not scanned_angles[n]
            if scanned_angles[n] == (scanned_angles[n+1]-1):
                while scanned_angles[n] == (scanned_angles[n+1]-1):
                    n += 1
                    if abs(scanned_dists[n]-scanned_dists[n+1]) < dist_threshold:
                        if scanned_angles[n] not in mergable_object_angle:
                            mergable_object_angle+=(scanned_angles[n])
                            mergable_object_dist+=(scanned_dists[n])
                        mergable_object_angle+=(self, scanned_angles[n+1])
                        mergable_object_dist+=(self, scanned_dists[n+1])

            mid_merged_object_angle = int(len(mergable_object_angle) / 2)
            mid_merged_object_dist = int(len(mergable_object_dist) / 2)
            merged_objects_angle += mergable_object_angle[mid_merged_object_angle]
            merged_objects_dist += mergable_object_dist[mid_merged_object_dist]

        scan_msg = ScannedObj()
        scan_msg.angle = merged_objects_angle
        scan_msg.dist = merged_objects_dist

        scan_msgs.scannedObjList.append(scan_msg)
        self.scanned_obj_pub.publish(scan_msgs)


if __name__ == '__main__':

    laser = LaserNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
