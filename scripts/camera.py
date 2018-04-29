import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from matplotlib import pyplot as plt
import numpy as np

class Camera:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_window = "Camera Input"
	self.colors_window = "Detected Colors"
        self.image_data = None
	
	self.histograms_window = "histograms"

    def show(self,img_msg=None):
        """
        TODO: feat_cameracallback, priority 4
            Move the time sync functions from the playnode here. The camera
            should'nt show up automatically.
        """
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        image = cv2.flip(image,-1)
        self.image_data = image
        cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
	self.detect_color(self.image_data)

    def callback(self, img_msg=None,laser_msg=None):
        """
        TODO: See show. This should be the new callback function. Save some of
        the relevant data in an attribute.
        """


    def show_histogram(self,image=None):
        """
        TODO: feat_det_color, priority 4
            Extract the color from img_msg and fill the the array with it and
            return an array with the same dimensions, but with enum color
            names. Relevant is green, yellow, blue.
        """
	color = ('b','g','r')
	for i,col in enumerate(color):
	  histr = cv2.calcHist([image],[i],None,[256],[0,256])
	  plt.plot(histr,color = col)
 	  plt.xlim([0,256])
	plt.ion()	#interactive mode, otherwise .show holds until window is closed
	plt.show()
	
    def detect_color(self, image=None):
	# values are RGB, testing for green
	lower = np.array([0,0,0])
	upper = np.array([0,255,0])
	
	mask = cv2.inRange(image, lower, upper)
	output = cv2.bitwise_and(image, image, mask = mask)
	cv2.imshow(self.colors_window, output)
