import random as rd
from time import sleep
import rospy
from geometry_msgs.msg import Twist

from camera import Camera
from laser import Laser

class Player(object):
    def __init__(self, vel_pub=None):
        self.laser = Laser()
        self.camera = Camera()
        self.vel_pub = vel_pub

    def run(self, laser_msg=None):
        """
        This is the laserscan callback function. It will be called in a loop
        with the rospy rate.
        TODO: fix_parameters, priority 5
            Play around the simulation and set a better distance and slice in
            the the lasers obstacle position.
        """
        if laser_msg is not None:
            if self.laser.save(self.laser.checkReliability(laser_msg)):
                self.laser.average_data = self.laser.processData(self.laser.data)
                self.laser.data = [] #cleans data in memory
        distance = 1.1
        self.avoid_obstacle(distance)
        self.detect_component(2)

    def forward(self,speed=0):
        if speed > 0:
            self.__set_velocities(speed, 0)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def turnRight(self,speed=0):
        if speed > 0:
            self.__set_velocities(0, -speed)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def turnLeft(self,speed=0):
        if speed > 0:
            self.__set_velocities(0, speed)
        else:
            raise ValueError("Invalid speed or set to 0!\n")

    def stop(self):
        self.__set_velocities(0,0)

    def avoid_obstacle(self,distance=None):
        """
        TODO: feat_better_avoid, priority 5
            Make the chance of turning 50% and forward movement 50%.
            Also play around a bit with the velocities and try to set better
            ones. Also add sleep for a sec or half a sec, so it moves a bit
            slower.
        """
        if distance > 0:
            detected_obj = self.laser.obstacle_position(distance)
            phi_view = 30
            for data in detected_obj:
                range_obj, phi_obj = data
#                rospy.loginfo("Obstacle detected! \nDistance: {} \
#                                                  \nDegrees: {} \
#                                                  ".format(range_obj,phi_obj))
                if abs(phi_obj) < phi_view:
                    if phi_obj > 0:
                        rospy.loginfo("Avoiding obstacle, turning right!")
                        self.turnRight(1)
                    else:
                        rospy.loginfo("Avoiding obstacle, turning left!")
                        self.turnLeft(1)

            rospy.loginfo("No obstacle ahead! Moving randomly!")
            if rd.random() < 0.66:
                self.forward(0.1)
            elif rd.random() < 0.81:
                self.turnRight(0.5)
            else:
                self.turnLeft(0.5)
        else:
            raise ValueError("Obstacle distance can't be negative!\n")

    def detect_component(self, distance=0):
        if distance > 0:
            detected_objs = self.laser.obstacle_position(distance)
            camera_objs = self.camera.objects

            for camera_obj in camera_objs:
                ranges,area,color=camera_obj
                x1,x2 = ranges
                phi1 = 11*(x1-30)/90-55
                phi2 = 11*(x2+30)/90-55
                rospy.loginfo("x1:{},x2:{}".format(x1,x2))
                rospy.loginfo("phi1:{},phi2:{}".format(phi1,phi2))
                for laser_obj in detected_objs:
                    distance,laser_phi = laser_obj
                    if abs(laser_phi)<50:
                        rospy.loginfo("{}".format(laser_obj))
                    if ((laser_phi>=phi1) and (laser_phi<=phi2)):
                        if color != "G" and area>10000:
                            if color == "Y":
                                rospy.loginfo("Detected the yellow goal!")
                            else:
                                rospy.loginfo("Detected the blue goal!")
                        elif color == "G":
                            rospy.loginfo("Detected a pole in {} m distance and {} degrees".format(distance,laser_phi))
                        elif color == "Y":
                            rospy.loginfo("Detected a yellow puck in {} m distance and {} degrees!".format(distance,laser_phi))
                        elif color == "B":
                            rospy.loginfo("Detected a blue puck in {} m distance and {} degree!".format(distance,laser_phi))

    def __set_velocities(self,linear=0, angular=None):
        if linear >= 0 and angular is not None:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.vel_pub.publish(msg)
        else:
            exit(-1)
