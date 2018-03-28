class Laser:
    def __init__(self):
        self.data = None

    def slice(self,p=None):
        if p is not None:
            if self.getDataSize(self.data)<p:
                raise ValueError("p is bigger than raw_laser_data size!\n")
                exit(-1)
            return self.data[p:-p]
        else:
            raise ValueError("Passed wrong argument to function!\n")
            exit(-1)

    def save(self,laser_msg=None):
        if not isinstance(laser_msg, Laser) and laser_msg is not None:
            self.data = laser_msg.ranges
            return(laser_msg.ranges)

    def getDataSize(self, laser_data):
        if laser_data is not None:
            sum =0
            for data in laser_data:
                sum = sum+1
            return sum
        else:
            raise ValueError("Couldn't compute laser data size!\n")

    def obstacle_position(self,threshhold=0):
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
