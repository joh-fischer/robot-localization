#!/usr/bin/env python
# @Author:  Johannes S. Fischer

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from util import *
from math import *
import random

class Robot:
    def __init__(self, gps_noise, sense_noise, world):
        rospy.loginfo("Initialize robot")
        self.world = world
        # true position
        self.trueX = 0
        self.trueY = 0
        self.trueOrientation = 0
        # estimated position
        self.x = 0
        self.y = 0
        self.orientation = 0

        # known position in the beginning
        self.recvGPS()
        self.setPosition(self.trueX, self.trueY, self.trueOrientation)
        
        # noise for the estimation
        self.gps_noise = gps_noise
        self.sense_noise = sense_noise

    def prettyPrint(self):
        print("Estimate:\t(%.2f, %.2f) \t Orientation: %.2f"%(self.x, self.y, self.orientation))
        print("Truth: \t\t(%.2f, %.2f) \t Orientation: %.2f"%(self.trueX, self.trueY, self.trueOrientation))

    def setTruePosition(self, x, y, orientation):
        self.trueX = x
        self.trueY = y
        self.trueOrientation = orientation
    
    def setPosition(self, x, y, orientation):
        if (abs(x) > self.world.size/2):
            print("X out of bounds, robot not moving")
            return False
        if (abs(y) > self.world.size/2):
            print("Y out of bounds, robot not moving")
            return False
        self.x = x
        self.y = y
        self.orientation = orientation
        return True

    def move(self, turn, forward):
        rospy.loginfo("Move robot: turn=%.3f, forward=%.3f"%(turn, forward))
        if (forward < 0):
            raise ValueError("Only forward kinematics allowed!")
        # orientation
        orientation = self.orientation + float(turn)
        # has to be within range [-pi, pi]
        orientation = (orientation + pi) % (2 * pi) - pi
        #print("Orientation: %.4f"%orientation)
        # moving forward
        distance = float(forward)
        x = self.x + (cos(orientation) * distance)
        y = self.y + (sin(orientation) * distance)
        # set values
        setPositionSuccess = self.setPosition(x, y, orientation)
        if (setPositionSuccess):
            self.rotate(1.0, turn)
            self.go(1.0, forward)

    def sense(self):
        self.recvGPS()
        rospy.loginfo("Sensing the landmark distances")
        allDistances = []
        for lm in self.world.landmarks:
            # get distance and add noise
            distance = getDistance( (self.trueX, self.trueY), lm) + random.gauss(0.0, self.sense_noise)
            allDistances.append( distance )
        return allDistances
    
    def recvGPS(self):
        rospy.loginfo("Receiving GPS data")
        # receive one message from the odometry
        data = rospy.wait_for_message("/ground_truth_odom", Odometry)
        # get orientation
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        _,_, orientation = euler_from_quaternion(x, y, z, w)
        # get position
        x_pos = data.pose.pose.position.x
        y_pos = data.pose.pose.position.y
        # set true values
        self.setTruePosition(x_pos, y_pos, orientation)

    def go(self, speed, distance):
        print("Go forward: speed=%s, distance=%s"%(speed, distance))
        if (distance == 0):
            print("Moving forward finished...")
            return
        velocity_publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1000)

        # set movement
        vel_msg = VelMsg()
        vel_msg.setForward(speed)

        while not rospy.is_shutdown():
            # Setting current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            # guarantee that it is non-zero
            while (not t0 > 0):
                t0 = rospy.Time.now().to_sec()
            current_distance = 0
            # loop to move in a specified distance
            while (current_distance < distance):
                # publish velocity
                velocity_publisher.publish(vel_msg.msg)
                # take actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # calculate distance of the pose
                current_distance = speed * (t1-t0)

            # stop robot
            print("Moving forward finished...")
            vel_msg.setForward(0)
            velocity_publisher.publish(vel_msg.msg)
            rospy.sleep(1.5)
            break

    def rotate(self, speed, turn):
        print("Rotate: speed=%s, turn=%.3f"%(speed, turn))
        velocity_publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1000)
        if (turn == 0):
            print("Turning finished...")
            return
        # set movement
        vel_msg = VelMsg()

        # check turning direction
        if (turn < 0):
            turn = abs(turn)
            vel_msg.setTurn(-speed)
        else:
            vel_msg.setTurn(speed)
        
        while not rospy.is_shutdown():
            # Setting current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            # guarantee that it is non-zero
            while (not t0 > 0):
                t0 = rospy.Time.now().to_sec()
            current_turn = 0
            # loop to move in a specified distance
            while(current_turn < turn):
                velocity_publisher.publish(vel_msg.msg)
                t1 = rospy.Time.now().to_sec()
                current_turn = speed * (t1-t0)
                #print("Turn: %.3f | Speed: %.3f | Time: %.3f"%(current_turn, speed, (t1-t0)))
            # stop robot
            print("Turning finished...")
            vel_msg.setTurn(0)
            velocity_publisher.publish(vel_msg.msg)
            rospy.sleep(1.5)
            break