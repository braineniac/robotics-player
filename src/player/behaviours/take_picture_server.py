#!/usr/bin/env python

import rospy
import random
import actionlib

from player.msg import TakePictureAction, TakePictureFeedback, TakePictureResult

class TakePictureServer:

    feedback = TakePictureFeedback()
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer("take_picture", TakePictureAction, self.execute, False)
        self.server.start()
        rospy.loginfo("initialised take picture server")

    def check_preempt(self):
        if self.server.is_preempt_requested():
            rospy.loginfo("I was preempted")
            self.server.set_preempted()
            return True
        
    def execute(self, goal):
        rospy.loginfo("got goal")
        result = TakePictureResult()
        
        self.feedback.message = "orienting to object"
        self.server.publish_feedback(self.feedback)
        rospy.sleep(0.5)
        if self.check_preempt():
            return
        
        self.feedback.message = "focusing camera"
        self.server.publish_feedback(self.feedback)
        rv = random.random()
        rospy.loginfo(rv)
        if rv < 0.5:
            result.blurry = True
        else:
            result.blurry = False

        rospy.loginfo("result is: {}".format(result))

        rospy.sleep(0.5)

        if self.check_preempt():
            return
        
        self.feedback.message = "taking picture"
        self.server.publish_feedback(self.feedback)
        rospy.sleep(1)
        if self.check_preempt():
            return

        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node("take_picture_server")
    server = TakePictureServer()
    
    rospy.spin()
