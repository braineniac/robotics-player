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
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
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
	# values are HSV, using hue+-10 for defining a color. Also while in HSV the
	# range of hue is 0-360 degrees, openCV uses hue/2 to fit the value into an int.

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
	
	maskGB = cv2.bitwise_or(maskG, maskB) #combine 3 masks into one
	mask = cv2.bitwise_or(maskGB, maskY)

	output = cv2.bitwise_and(image, image, mask = mask)
	cv2.imshow(self.colors_window, output)

