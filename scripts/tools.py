import rospy

def rosprint(string=None):
    if rosprint:
        rospy.loginfo("{}".format(string))
