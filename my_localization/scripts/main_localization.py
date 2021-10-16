#!/usr/bin/env python
# @Author:  Johannes S. Fischer

from Robot import Robot
from RobotDummy import RobotDummy
from World import World
from ParticleFilter import ParticleFilter

import rospy
import time
import random
from math import pi

if __name__ == "__main__":
    """
    1. distribute particles
    2. move robot
    3. sense distance
    4. move particles
    5. calculate weights
    6. resample
    7. plot
        - true position, orientation
        - resampled particles
        - old particles
    """
    print("**** Start localization ****")
    # start timing
    time_start = time.time()

    # choose steps
    steps = 10
    # choose number of particles
    nParticles = 1000
    
    # set world
    size = 20.0
    landmarks = [[-10.0, -10.0], [10.0, 10.0], [-10.0, 10.0], [10.0, -10.0], [0, 10]]
    world = World(size, landmarks)

    # set particle filter
    particleFilter = ParticleFilter(nParticles, 0.05, 0.05, 5.0, world)
    print("World size: %s"%particleFilter.WORLD.size)

    # set robot
    speed = 1
    distance = 1

    turning_pattern = [0, 1.57, 1.57, 1.0, -2.57, -1.57, 0.6, -0.4]
    forward_pattern = [3, 2, 6, 4, 3, 2, 3, 2]

    steps = len(turning_pattern)

    # start new node
    rospy.init_node("main_localization", anonymous=True)
    # initialize robot
    myRobot = Robot(0.05, 0.05, world)

    # set robot
    #myRobot = RobotDummy(world)
    myRobot.prettyPrint()
    for t in range(steps):
        rospy.loginfo("********* Step %s *********"%t)
        myRobot.prettyPrint()
                
        # move the robot in random direction
        #move_turn = random.uniform(-1.0, 1.0) * pi
        #move_forward = random.random() * 2
        move_turn = turning_pattern[t]
        move_forward = forward_pattern[t]
        
        myRobot.move(move_turn, move_forward)

        """ 1. simulate the robot motion for each particle """
        particles = particleFilter.simulateMotion(move_turn, move_forward)

        """ 2. weight different particles, based on sensor data """
        sensed_distances = myRobot.sense()
        weights = particleFilter.weightParticles(sensed_distances)

        """ 3. resample, with more higher weighted particles """
        particles_sample = particleFilter.resampleParticles()

        """ 4. estimate position based on particles """
        estimated_position = particleFilter.estimatePosition()
        
        #particleFilter.showParticles()

        # visualize
        particleFilter.visualize(myRobot, t, particles, estimated_position)

    # getting timing
    time_end = time.time()
    running_time = time_end - time_start
    print("Program run %.4f seconds for %s steps and %s particles"%(running_time, steps, particleFilter.n))
            