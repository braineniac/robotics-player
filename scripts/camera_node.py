#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
import tf2_ros

import tf2_sensor_msgs
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from team3_msgs.msg import KinectObj, KinectObjs
from tools import rosprint


class CameraNode:

    def __init__(self):
        # inialising the node and publishers/subscribers
        rospy.init_node("camera_node", anonymous=True)
        rospy.loginfo("Initialised camera node!")
        self.sim_env = rospy.get_param('sim_env')
        rosprint("Is our world a simulation? {}".format(self.sim_env))
        self.rgb_sub = rospy.Subscriber("kinect/rgb/image_raw", Image, self.rgb_cb)
        self.point_sub = rospy.Subscriber("camera_depth", PointCloud2, self.pt_cb)
        # parameters
        self.bridge = CvBridge()
        self.image_window = "Camera Input"
        self.color_window = "Detected Colors"
        self.image_data = None
        self.objects = KinectObjs().kinectObjList
        self.pub = rospy.Publisher("kinect_objs", KinectObjs, queue_size=1)
#        self.pub2 = rospy.Publisher("kinect_obj", KinectObj, queue_size=1)
        #self.pubtest = rospy.Publisher("detected_points", PointCloud2, queue_size=1000)  # detected points test

        # keeps node from exiting

        self.pc_data = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        rospy.spin()

    def pt_cb(self, pt_msg):
        self.pc_data = pt_msg
#        rospy.loginfo(self.pc_data.point_step)

    def rgb_cb(self, img_msg=None):
        # this method initializes everything and ends with publishing detected objects

        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        #image = cv2.flip(image, -1) # only needed for usb camera, not for kinect one
        self.image_data = image
        #cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
        self.blurfilter(self.image_data)

        self.Objs = KinectObjs()

        self.detect_color(self.image_blurred)
        #        rospy.loginfo("Amount of green blobs:")
        self.detect_contours_and_pixels(self.color_data_G, "G")
        #        rospy.loginfo("Amount of blue blobs:")
        self.detect_contours_and_pixels(self.color_data_B, "B")
        #        rospy.loginfo("Amount of yellow blobs:")
        self.detect_contours_and_pixels(self.color_data_Y, "Y")

        if self.pc_data is not None:
            self.Objs.header = self.pc_data.header
            self.pub.publish(self.Objs)

    def blurfilter(self, image=None):
        self.image_blurred = cv2.GaussianBlur(image, (7,7), 6)

    def detect_color(self, image=None):
        # values are HSV, using hue+-10 for defining a color. Also while in HSV the
        # range of hue is 0-360 degrees, openCV uses hue/2 to fit the value into an int.
        if self.sim_env:
            lowerG = np.array([50, 0, 0])
            upperG = np.array([70, 255, 255])

            lowerB = np.array([110, 0, 0])
            upperB = np.array([130, 255, 255])

            lowerY = np.array([20, 0, 0])
            upperY = np.array([40, 255, 255])

        else:
            lowerG = np.array([60, 20, 0])
            upperG = np.array([73, 255, 255])

            lowerB = np.array([86, 20, 0])
            upperB = np.array([100, 255, 255])

            lowerY = np.array([20, 20, 150])
            upperY = np.array([34, 255, 255])
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


        outputG = cv2.bitwise_and(image, image, mask=maskG)
        outputB = cv2.bitwise_and(image, image, mask=maskB)
        outputY = cv2.bitwise_and(image, image, mask=maskY)

        self.color_data_G = outputG
        self.color_data_B = outputB
        self.color_data_Y = outputY


    def detect_contours_and_pixels(self, color_data=None, color=None):
        # 80 degrees whole view, about half of the pole in 0.68m
        # using FindContours(), which requires binary image

        img_gray = cv2.cvtColor(color_data, cv2.COLOR_BGR2GRAY)
        img_bin = cv2.threshold(img_gray, 10, 255, cv2.THRESH_BINARY)[1]
     #   cv2.imshow(self.color_window, img_bin)
        contours = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        # contours is a list of sequences, so number of objects detected iterates through list
        # (because size() or len() crapped out when only a single object was present)

        self.contour_squares = []

        for contour in contours:
            x_values = []
            y_values = []
            pxl_amount = 0
            # would be prettier with cv2.BoundingRSect
            #rospy.loginfo(cv2.contourArea(contour))
            if cv2.contourArea(contour) >= 800:     # only use contours containing more than x pixels
                for elem in contour:
                    x_values.append(elem[0][0])
                    y_values.append(elem[0][1])
                height = max(y_values)-min(y_values)
                width = max(x_values)-min(x_values)
#                area = cv2.contourArea(contour)
                for i in range(min(x_values), min(x_values) + width):
                    for j in range(min(y_values), min(y_values) + height):
                        if img_bin[j, i] == 255:
                            pxl_amount = pxl_amount + 1.0
                self.contour_squares.append((min(x_values), min(y_values), width, height, pxl_amount/(height*width)))
                cv2.rectangle(self.image_data, (min(x_values), min(y_values)), (min(x_values)+width, min(y_values)+height), (0,255,0), 2)
                #rospy.loginfo("pixels: {}, density: {}".format(height*width, pxl_amount/(height*width)))


        cv2.imshow(self.image_window, self.image_data)

        object_pixels = []
        for square in self.contour_squares:
            if square[4] > 0.5:
                pixels = [square[0]+square[2]//2,square[1]+square[3]//2]
                object_pixels.append(pixels)
        self.extract_objects(object_pixels, color)


    def extract_objects(self, object_pixels=None, color=None):

        for i in object_pixels:
            if self.pc_data is None:
                return
            object_points = list(pc2.read_points(self.pc_data, skip_nans=True, field_names=("x", "y", "z"), uvs=[i]))
            #rospy.loginfo(object_points[0])
            Obj = team3_msgs.msg.KinectObj()
            # transforming coordinates
            Obj.x = -object_points[0][2]
            Obj.y = object_points[0][1]
            Obj.z = object_points[0][0]
            Obj.delta_x = 0.0
            Obj.delta_y = 0.0
            Obj.delta_z = 0.0
            Obj.color = color
            self.Objs.kinectObjList.append(Obj)

        # rospy.loginfo("{} objects of color {} detected".format(len(object_pixels), color))

if __name__ == '__main__':

    camera_node = CameraNode()

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()

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


        """
        #for use in detect_contours
        object_pixels = []
        for square in self.contour_squares:
            pixels = []
            for i in range(square[0],square[0]+square[2]):
                for j in range(square[1],square[1]+square[3]):
                    if img_bin[j,i] == 255:
                        pixels.append([i,j])
            object_pixels.append(pixels)
        self.extract_objects(self.contour_squares, color, object_pixels)

    def extract_objects(self, contour_squares = None, color = None, object_pixels = None):
        object_pointclouds = []
        colors = {"G":60,"B":120,"Y":30}

        for object in object_pixels:
            distances = []
            rospy.loginfo(np.shape(object))
            rospy.loginfo(len(object))
            for i in range(0,len(object)):
                #distances.append(self.depth_data[object[i,0],object[i,1]])
                rospy.loginfo(self.depth_data)
        """

        """
        for object in object_pixels:
            object_points = list(pc2.read_points(self.pc_data, skip_nans=True, field_names=("x", "y", "z", "r", "g", "b"), uvs=object))

            fields = self.pc_data.fields[0:3]
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'robot1/kinect_depth_optical_frame'
            msg = pc2.create_cloud(header, fields, object_points)
            msg.height = 1
            msg.width = len(object)
            rospy.loginfo(msg.width)
            self.pubtest.publish(msg)
        """
