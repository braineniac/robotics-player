import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError

class Camera:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_window = "Camera Input"

    def show(self,img_msg=None, laser_msg=None):
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        image = cv2.flip(image,-1)
        cv2.imshow(self.image_window, image)
        cv2.waitKey(10)
