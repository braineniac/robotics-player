#!/usr/bin/env python

import rospy
import random
import actionlib

from player.msg import *
from std_msgs.msg import Bool
from player import rosprint

class RandomDriveServer:

    feedback = RandomDriveFeedback()
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer("drive_randomly", RandomDriveAction, self.execute, False)
        self.depth_sub = rospy.Subscriber("detected_objs", DetectedObjs, self.depth_sub_cb)
        self.move_pub = rospy.Publisher("cmd_move", CmdMove, queue_size=1)
        self.OR_pub = rospy.Publisher("OR_execution", Bool,queue_size=1)
        self.server.start()

        self.freshest_detection = None

        rospy.loginfo("initialised drive randomly server")

    def execute(self, goal):
        # pretend to drive around randomly and detect objects or obstacles
        result = RandomDriveResult()
        rate = rospy.Rate(1) # tick twice per second
        count = 0
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo("I was preempted")
                self.server.set_preempted()
                break
            #rosprint("Drive randomly server is running!")
            obstacle = False
            detect_stuff = Bool()
            detect_stuff.data = 1
            self.OR_pub.publish(detect_stuff)

            if self.freshest_detection:
                for object in self.freshest_detection.detectedObjList:
                    if (object.x < 1 and object.x > 0 and object.y < 0.5 and object.y > -0.5) and object.id != "Y_goal" and object.id != "B_goal":
                        rospy.loginfo("obstacle {} at x: {}, y:{}, avoiding".format(object.id, object.x, object.y))
                        obstacle = True
                self.freshest_detection = None
            #rv = random.random()

            movement = CmdMove()


            if obstacle:
                result.obstacle_found = True
                self.feedback.message = "found obstacle"
                self.server.publish_feedback(self.feedback)
                self.server.set_succeeded(result)
                movement.direction = "stop"
                movement.duration = 0.4
                movement.speed = 0
                self.move_pub.publish(movement)
                break
            else:
                choices = ["fwd", "fwd", "cw", "ccw"]
                movement.direction = random.choice(choices)
                movement.duration = random.uniform(0.3, 3)
                movement.speed = random.uniform(0.05,0.2)
                self.move_pub.publish(movement)
                rospy.loginfo("moving randomly with direction:{}, duration:{},speed:{}".format(movement.direction,movement.duration,movement.speed))

                rospy.sleep(0.5)
                movement.direction = "stop"
                self.move_pub.publish(movement)

                count += 1
                self.feedback.message = "driving randomly"
                self.server.publish_feedback(self.feedback)

            rate.sleep()

    def depth_sub_cb(self, message):
        if message:
            self.freshest_detection = message



if __name__ == '__main__':
    rospy.init_node("drive_randomly_server")
    server = RandomDriveServer()
    
    rospy.spin()
