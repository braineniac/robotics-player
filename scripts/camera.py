import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError

class Camera:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_window = "Camera Input"

    def show(self,img_msg=None, laser_msg=None):
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
        cv2.imshow(self.image_window, image)
        cv2.waitKey(10)

    def callback(self, img_msg=None,laser_msg=None):
        """
        TODO: See show. This should be the new callback function. Save some of
        the relevant data in an attribute.
        """

    def detect_color(self,img_msg=None):
        """
        TODO: feat_det_color, priority 4
            Extract the color from img_msg and fill the the array with it and
            return an array with the same dimensions, but with enum color
            names. Relevant is green, yellow, blue.
        """