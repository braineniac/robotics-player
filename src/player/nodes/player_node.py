#!/usr/bin/env python

import rospy
import random as rd
import smach
from smach_ros import SimpleActionState, IntrospectionServer
from player.msg import *
from player import rosprint

class PlayerNode:
    def __init__(self):
        #inialising the node and publishers/subsribers
        rospy.init_node("player_node",anonymous=True)
        rospy.loginfo("Player node initialised.")

        self.move_pub = rospy.Publisher("cmd_move", CmdMove, queue_size=1000)
        self.odom_sub = rospy.Subscriber("pose_deltas", DeltaPose, self.odom_cb)

        self.trans = [0,0,0]
        self.rot_euler = [0,0,0]

    def run_smach(self):
        sm = smach.StateMachine(outcomes=["preempted", "aborted", "succeeded"])

        with sm:
            @smach.cb_interface(outcomes=['map_build_successeded', 'map_build_failed'])
            def build_map_cb(userdata,status,result):
                rosprint("Result of build_map:{}".format(result))
                if result.message == "build_map_failed":
                    return "map_build_failed"
                elif result.message == "map_build_successeded":
                    return "map_build_successeded"

            smach.StateMachine.add("BUILD_MAP",
                    SimpleActionState("build_map",
                                      BuildMapAction,
                                      result_cb=build_map_cb), #???
                    transitions={"map_build_failed": "BUILD_MAP",
                                 "map_build_succeeded": "MOVE"})
            smach.StateMachine.add("MOVE",
                    SimpleActionState("build_map",
                                      MoveAction,
                                      goal_slots=["ccw",10,10]),
                    transitions={"succeeded": "BUILD_MAP"})

        sis = IntrospectionServer("player_node", sm, "/SM_ROOT")
        sis.start()

        outcome = sm.execute()
        rospy.spin()
        sis.stop()

    def odom_cb(self, odom_msg):
        self.trans[0] += odom_msg.delta_x
        self.trans[1] += odom_msg.delta_y
        self.rot_euler[2] += odom_msg.delt_phi

    def move(self, direction, duration=0.5, speed=0.1):
        """
        Sends CmdMove message to cmd_move topic. Direction: "fwd" = forward, "cw" = clockwise, "ccw" = counterclockwise, "stop" = stop. Duration in seconds. Speed in m/s.
        """
        msg = CmdMove()
        if direction in ["fwd","cw","ccw","stop"]:
            msg.direction = direction
            if direction == "stop":
                self.move_pub.publish(msg)
        else:
            raise ValueError("Invalid direction specifier! (Valid specifiers: fwd, cw, ccw, stop)\n")
        if duration > 0:
            msg.duration = duration
        else:
            raise ValueError("Duration is negative, 0 or unspecified!\n")
        if speed > 0:
            msg.speed = speed
        else:
            raise ValueError("Speed is negative, 0 or unspecified!\n")
        self.move_pub.publish(msg)

    def avoid_obstacle(self,distance=None):
        """
        Sends CmdMove message to cmd_move topic. Direction: "fwd" = forward, "cw" = clockwise, "ccw" = counterclockwise. Duration in seconds. Speed in m/s.
        """
        if distance > 0:
            detected_obj = self.laser.obstacle_position(distance)
            phi_view = 30
            for data in detected_obj:
                range_obj, phi_obj = data
                if abs(phi_obj) < phi_view:
                    if phi_obj > 0:
                        rosprint("Avoiding obstacle, turning right!")
                        self.move("cw",1,1)
                    else:
                        rosprint("Avoiding obstacle, turning left!")
                        self.move("ccw",1,1)

            rosprint("No obstacle ahead! Moving randomly!")
            if rd.random() < 0.66:
                self.move("fwd",1,0.1)
            elif rd.random() < 0.81:
                self.move("cw",1,0.5)
            else:
                self.move("ccw",1,0.5)
        else:
            raise ValueError("Obstacle distance can't be negative!\n")


if __name__ == '__main__':

    player = PlayerNode()
    player.run_smach()
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 10 Hz loop
        loop_rate.sleep()
