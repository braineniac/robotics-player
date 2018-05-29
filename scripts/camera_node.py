#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
from team3_msgs.msg import KinectObj, KinectObjs

from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import tf2_ros
import tf2_sensor_msgs
from tools import rosprint
import sensor_msgs.point_cloud2 as pc2


class CameraNode:

    def __init__(self):
        # inialising the node and publishers/subscribers
        rospy.init_node("camera_node", anonymous=True)
        rospy.loginfo("Camera node initialised.")
        self.rgb_sub = rospy.Subscriber("kinect/rgb/image_raw", Image, self.rgb_cb)
        self.depth_sub = rospy.Subscriber("kinect/depth/points", PointCloud2, self.pt_cb)
        # parameters
        self.bridge = CvBridge()
        self.image_window = "Camera Input"
        self.color_window = "Detected Colors"
        self.image_data = None
        self.histograms_window = "histograms"
        self.objects = KinectObjs().kinectObjList
        self.pub = rospy.Publisher("camera_objs", KinectObjs, queue_size=1000)
        self.pubtest = rospy.Publisher("detected_points", PointCloud2, queue_size=1000)  # detected points test

        # keeps node from exiting
        rospy.spin()

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def pt_cb(self, pt_msg):
        """
        TODO: replace range detection, base it on pointclouds.
        """
        uvstest = []
        for i in range(310, 330):
            for j in range(238, 242):
                uvstest.append([i, j])
        testpoints = list(pc2.read_points(pt_msg, skip_nans=True, field_names=("x", "y", "z"), uvs=uvstest))
        fields = pt_msg.fields[0:3]
        #	rospy.loginfo(fields)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'robot1/kinect_depth_optical_frame'
        msg = pc2.create_cloud(header, fields, testpoints)
        msg.height = 4
        msg.width = 20
        self.pubtest.publish(msg)

        #	ptc2 = pcl.load(ptc)
        #	rospy.loginfo(type(ptc2))
        #	self.pointclouddata(ptc)
        pass

    def pointclouddata(self, pointcloud):
#        rospy.loginfo("pointcloud type: {}".format(type(pointcloud)))
 #       rospy.loginfo(pointcloud)
        #	for i in pointcloud:
        #	    rospy.loginfo(("x","y","z"))
        pass

    def rgb_cb(self, img_msg=None):
        self.objects = []
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        image = cv2.flip(image, -1)
        self.image_data = image
        #cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
        self.filter(self.image_data)
        self.detect_color(self.image_blurred)
        #        rospy.loginfo("Amount of green blobs:")
        self.detect_contours(self.color_data_G, "G")
        #        rospy.loginfo("Amount of blue blobs:")
        self.detect_contours(self.color_data_B, "B")
        #        rospy.loginfo("Amount of yellow blobs:")
        self.detect_contours(self.color_data_Y, "Y")
        self.pub.publish(self.objects)

    def filter(self, image=None):
        self.image_blurred = cv2.medianBlur(image, 7)

    def detect_color(self, image=None):
        # values are HSV, using hue+-10 for defining a color. Also while in HSV the
        # range of hue is 0-360 degrees, openCV uses hue/2 to fit the value into an int.
        """
        TODO: polish into method which takes two hue values as input and
        doesn't contain everything thrice for coding practice.
        """
        lowerG = np.array([50, 0, 0])
        upperG = np.array([70, 255, 255])

        lowerB = np.array([110, 0, 0])
        upperB = np.array([130, 255, 255])

        lowerY = np.array([20, 0, 0])
        upperY = np.array([40, 255, 255])
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

        nonz = np.nonzero(maskG)
        self.nonz_iterable = np.transpose(nonz)

  #      rospy.loginfo(np.shape(maskG))

        outputG = cv2.bitwise_and(image, image, mask=maskG)
        outputB = cv2.bitwise_and(image, image, mask=maskB)
        outputY = cv2.bitwise_and(image, image, mask=maskY)

        self.color_data_G = outputG
        self.color_data_B = outputB
        self.color_data_Y = outputY
#        cv2.imshow(self.color_window, self.color_data_Y)

    def detect_contours(self, image=None, color=None):
        # 80 degrees whole view, about half of the pole in 0.68m
        # using FindContours(), which requires binary image
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.threshold(img_gray, 0.0001, 255, cv2.THRESH_BINARY)[1]

        contours = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        # contours is a list of sequences, so number of objects detected iterates through list
        # (because size() or len() crapped out when only a single object was present)

        self.contour_squares = []

        for contour in contours:
            x_values = []
            y_values = []
            for elem in contour:
                x_values.append(elem[0][0])
                y_values.append(elem[0][1])
            height = max(y_values)-min(y_values)
            width = max(x_values)-min(x_values)
#            area = cv2.contourArea(contour)
            self.contour_squares.append((min(x_values),min(y_values),width,height))
            cv2.rectangle(self.image_data, (min(x_values),min(y_values)), (min(x_values)+width,min(y_values)+height),(0,255,0),2)
   #     rospy.loginfo(self.contour_squares)
        cv2.imshow(self.image_window, self.image_data)
#        rospy.loginfo("Objects:{}".format(self.objects))
#        rospy.loginfo("Contour size: {}".format(len(contour_x_values)))

    """
        for border in contour_x_values:
            msg = KinectObj()
            msg.lower = np.amin(border)
            msg.upper = np.amax(border)
            msg.area = area
            msg.color = color
            self.objects.append(msg)
    """


    # ========================================================================
    # unused but potentially useful (read: useless) code
    # ========================================================================

    def show_histogram(self, image=None):
        color = ('b', 'g', 'r')
        for i, col in enumerate(color):
            histr = cv2.calcHist([image], [i], None, [256], [0, 256])
            plt.plot(histr, color=col)
            plt.xlim([0, 256])
            plt.ion()  # interactive mode, otherwise .show holds until window is closed
            plt.show()
        """
	    pxlamount = np.count_nonzero(output)/3
	    rospy.loginfo("number of pixels detected: {}".format(pxlamount))
	    """


if __name__ == '__main__':

    camera_node = CameraNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
