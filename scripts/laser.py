class Laser:
    def __init__(self):
        self.data = []

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

    def checkReliability(slef, laser_msg=None, threshold=0.5):
        """
        Retursn only reliable ranges. Sets all unreliable values to nan.
        """
        if laser_msg is not None:
            for i in range(len(laser_msg.ranges)):
                if laser_msg.intensities[i] < threshold:
                    laser_msg.ranges[i] = float('nan')
            return laser_msg.ranges

    def save(self,laser_ranges=None):
        """
        Saves the read data from the scan into an attribute.
        """
        if not isinstance(laser_ranges, list) and laser_ranges is not None:
            if(len(self.data) > 5):
                return false
            else:
                self.data.append(laser_ranges)
                return true

    def average(self, laser_ranges):
        """
        TODO: feat_averaging, priority 5
            Implement an averaging function that saves the relevant laser scan
            data that are reliable(also from intensities), saves them in an
            attribute and after 5 calls, it sets the data attribute. But leave
            the save function here after you are done, just in case.
        """
        if laser_ranges is not None sinstance(laser_ranges, list):


        pass

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
            middle_raw = int(self.getDataSize(self.data)/2)
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
