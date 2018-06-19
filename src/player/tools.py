import rospy

def rosprint(string=None):
    if string:
        rospy.loginfo("{}".format(string))

def getDataSize(self, data=None):
    if data:
        sum =0
        for data in laser_data:
            sum = sum+1
        return sum
    else:
        raise ValueError("Couldn't compute laser data size!\n")
