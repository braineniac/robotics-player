#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
from team3_msgs.msg import KinectObj, KinectObjs

from std_msgs.msg import String
from sensor_msgs.msg import Image

class Camera:

    def __init__(self):
        #inialising the node and publishers/subsribers
        rospy.init_node("camera",anonymous=True)
        rospy.loginfo("Camera node initialised.")
        self.img_sub = rospy.Subscriber("kinect/rgb/image_raw", Image,self.show)
        self.depth_sub = rospy.Subscriber("camera_depth", self.callback)
        #parametrs
        self.bridge = CvBridge()
        self.image_window = "Camera Input"
        self.color_window = "Detected Colors"
        self.image_data = None
        self.histograms_window = "histograms"
        self.objects=[]
        self.pub = rospy.Publisher("camera_objs", KinectObjs)

        #keeps node from exiting
        rospy.spin()

    def show(self,img_msg=None):
        """
        TODO: feat_cameracallback, priority 4
            Move the time sync functions from the playnode here. The camera
            should'nt show up automatically.
        """
        self.objects = []
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        image = cv2.flip(image,-1)
        self.image_data = image
        cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
        self.detect_color(self.image_data)
#        rospy.loginfo("Amount of green blobs:")
        self.detect_contours(self.color_data_G, "G")
#        rospy.loginfo("Amount of blue blobs:")
        self.detect_contours(self.color_data_B, "B")
#        rospy.loginfo("Amount of yellow blobs:")
        self.detect_contours(self.color_data_Y, "Y")
        self.pub.publish(self.objects)

    def callback(self, img_msg=None,laser_msg=None):
        """
        TODO: See show. This should be the new callback function. Save some of
        the relevant data in an attribute.
        """

    def detect_color(self, image=None):
	# values are HSV, using hue+-10 for defining a color. Also while in HSV the
	# range of hue is 0-360 degrees, openCV uses hue/2 to fit the value into an int.
        """
        TODO: polish into method which takes two hue values as input and
        doesn't contain everything thrice for coding practice.
        """
        lowerG = np.array([50,0,0])
        upperG = np.array([70,255,255])

        lowerB = np.array([110,0,0])
        upperB = np.array([130,255,255])

        lowerY = np.array([20,0,0])
        upperY = np.array([40,255,255])
        """
        lowerbottomR = np.array([0,0,0])	#HSV ranges for red, should we need it
        upperbottomR = np.array([10,255,255])   #since red hue value is 0 we need 2 ranges
        lowertopR = np.array([170,0,0])
        uppertopR = np.array([180,255,255])
        """
        imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        maskG = cv2.inRange(imgHSV, lowerG, upperG)
        maskB = cv2.inRange(imgHSV, lowerB, upperB)
        maskY = cv2.inRange(imgHSV, lowerY, upperY)

        outputG = cv2.bitwise_and(image, image, mask = maskG)
        outputB = cv2.bitwise_and(image, image, mask = maskB)
        outputY = cv2.bitwise_and(image, image, mask = maskY)

        self.color_data_G = outputG
        self.color_data_B = outputB
        self.color_data_Y = outputY

    def detect_contours(self, image=None, color=None):
        #80 degrees whole view, about half of the pole in 0.68m
	# using FindContours(), which requires binary image
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.threshold(img_gray,0.0001,255,cv2.THRESH_BINARY)[1]

        contours=cv2.findContours(img_bin,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1]

	# contours is a list of sequences, so number of objects detected iterates through list
	# (because size() or len() crapped out when only a single object was present)
        contour_x_values=[]
        for contour in contours:
            x_values=[]
            for elem in contour:
                for x,y in elem:
                    x_values.append(x)
            area = cv2.contourArea(contour)
#            rospy.loginfo("{}".format(x_values))
            contour_x_values.append(x_values)
#        rospy.loginfo("Objects:{}".format(self.objects))
#        rospy.loginfo("Contour size: {}".format(len(contour_x_values)))
        for border in contour_x_values:
            msg = KinectObj()
            msg.lower = np.amin(border)
            msg.upper = np.amax(border)
            msg.area = area
            msg.color = color
            self.objects.kinectObjList.append(msg)
#        i=0
#        while contours:
#            del contours[0]
#            i = i + 1
#        rospy.loginfo("{} blobs".format(i))

#========================================================================
#unused but potentially useful code
#========================================================================


    def show_histogram(self,image=None):
        color = ('b','g','r')
        for i,col in enumerate(color):
            histr = cv2.calcHist([image],[i],None,[256],[0,256])
            plt.plot(histr,color = col)
            plt.xlim([0,256])
            plt.ion()	#interactive mode, otherwise .show holds until window is closed
            plt.show()
        """
	    pxlamount = np.count_nonzero(output)/3
	    rospy.loginfo("number of pixels detected: {}".format(pxlamount))
	    """
if __name__ == '__main__':

    camera = Camera()

    loop_rate = rospy.Rate(10)
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
