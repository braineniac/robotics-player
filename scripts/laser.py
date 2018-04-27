import numpy as np
import rospy

class Laser:
    def __init__(self):
        self.data = []
        self.average_data = []

    def slice(self,p=None):
        """
        TODO: feat_tools, priority 3
            Move slice and getDataSize to a new module called tools.py.
        """
        if p is not None:
            if self.getDataSize(self.data)<p:
                raise ValueError("p is bigger than raw_laser_data size!\n")
                exit(-1)
            return self.data[p:-p]
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
                if laser_msg.intensities[i] > threshold:
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

    def interpolateMissing(self, laser_ranges):
        """
        Returns list with interpolated values for NaNs in passed list.
        """
        for i,val in enumerate(laser_ranges):
            if np.isnan(val):
                left = None
                right = None
                offset = 0
                while(left == None):
                    try:
                        if(np.isnan(laser_ranges[i+offset])):
                            offset++
                        else:
                            left = laser_ranges[i+offset]
                    except IndexError:
                        left = np.nan

                offset = 0
                while(right == None):
                    try:
                        if(np.isnan(laser_ranges[i-offset])):
                            offset++
                        else:
                            right = laser_ranges[i-offset]
                    except IndexError:
                        right = np.nan

                if(np.isnan(left) and np.isnan(right)):
                    left = np.inf
                elif(np.isnan(left)):
                    left = right
                elif(np.isnan(right)):
                    right = left

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
        if self.data is None:
            return -5
        elif threshhold > 0:
            middle_raw = int(self.getDataSize(self.average_data)/2)
            laser_data = self.slice(middle_raw-15)
            middle_sliced = int(self.getDataSize(laser_data)/2)
            laser_middle_data =[laser_data[middle_sliced-1],
                    laser_data[middle_sliced],laser_data[middle_sliced+1]]
            laser_left_data= laser_data[:middle_sliced-1]
            laser_right_data = laser_data[middle_sliced+1:]
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
            raise ValueError("The threshhold can't be zero!\n");
            exit(-1)
