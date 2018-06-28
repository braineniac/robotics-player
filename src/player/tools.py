import rospy

def rosprint(string=None):
    if string is not None:
        rospy.loginfo("{}".format(string))

def getDataSize(data=None):
    if data:
        sum =0
        for elem in laser_data:
            sum = sum+1
        return sum
    else:
        raise ValueError("Couldn't compute laser data size!\n")
