#!/usr/bin/env python
# @Author:  Johannes S. Fischer

import time
from math import *
import random
from util import *
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy

""" Particle Filter class, which includes the particles class """
class ParticleFilter:
    # global variable world
    WORLD = -1
    
    def __init__(self, numberOfParticles, forward_noise, turn_noise, sense_noise, world):
        # set world as global variable
        ParticleFilter.WORLD = world
        # number of particles
        self.n = numberOfParticles
        # initialize weights to 1/n
        self.weights = [1.0/self.n] * self.n
        # create n number of particles
        particles = []
        for i in range(numberOfParticles):
            x = self.Particle()
            x.setNoise(forward_noise, turn_noise, sense_noise)
            particles.append(x)
        self.particles = particles

    def showParticles(self):
        print("Show all particles")
        for i in range(self.n):
            singleParticle = self.particles[i]
            print("Particle %s:  position = (%.2f, %.2f) \torientation = %.2f \tweight = %.2f"%(i, singleParticle.x, singleParticle.y, singleParticle.orientation, self.weights[i]))
    
    def simulateMotion(self, turn, forward):
        rospy.loginfo("Simulate motion for particles")
        particles_new = []
        for i in range(self.n):
            particles_new.append( self.particles[i].move(turn, forward))
        self.particles = particles_new
        
        return particles_new

    def weightParticles(self, sensor_data):
        rospy.loginfo("Weight the particles")
        # weight the particles based on the sensor data
        weights_new = []
        for i in range(self.n):
            weights_new.append( self.particles[i].getProbOfMeasurement( sensor_data ))
        # normalize weight weights
        weights_new = normalize(weights_new)
        self.weights = weights_new
        return weights_new

    def resampleParticles(self):
        rospy.loginfo("Resample particles")
        resampled_particles = []
        # resampling algorithm
        index = int(random.random() * self.n)
        beta = 0.0
        max_weights = max(self.weights)
        for i in range(self.n):
            # get random index
            beta += random.random() * 2.0 * max_weights
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index+1) % self.n
            # append random particles
            resampled_particles.append( self.particles[index] )
        self.particles = resampled_particles
        
        return resampled_particles
    
    def estimatePosition(self):
        rospy.loginfo("Estimate new position")
        all_x = []
        all_y = []
        all_orientation = []
        for idx, part in enumerate(self.particles):
            #print("X: %.3f \t Y: %.3f \t O: %.3f \t Weights: %.3f"%(part.x, part.y, part.orientation, self.weights[idx]))
            all_x.append( part.x * self.weights[idx] )
            all_y.append( part.y * self.weights[idx] )
            all_orientation.append( part.orientation )
        x_estimate = np.sum(all_x)
        y_estimate = np.sum(all_y)
        orientation_estimate = mean_circular_quantities(all_orientation, self.weights)
        print("Estimated position: (%.3f, %.3f)\nEstimated rotation: %.3f"%(x_estimate, y_estimate, orientation_estimate))
        return (x_estimate, y_estimate, orientation_estimate)

    def visualize(self, robot, step, old_particles, estimated_position):
        rospy.loginfo("Visualize particle filtering")
        markersize = 100
        # arrow
        scale = 0.2
        width = 0.1
        # plotting
        plt.figure("Robot", figsize=(20., 20.))
        plt.title('Particle filter, step ' + str(step))
        # draw coordinate grid for plotting
        grid = [(-ParticleFilter.WORLD.size/2)-1, (ParticleFilter.WORLD.size/2)+1, (-ParticleFilter.WORLD.size/2)-1, (ParticleFilter.WORLD.size/2)+1]
        plt.axis(grid)
        plt.grid(b=True, which='major', color='0.75', linestyle='--')
        plt.xticks([i for i in range(-int(ParticleFilter.WORLD.size/2)-1, int(ParticleFilter.WORLD.size/2)+1, 1)])
        plt.yticks([i for i in range(-int(ParticleFilter.WORLD.size/2)-1, int(ParticleFilter.WORLD.size/2)+1, 1)])
        # draw particles
        for old_part in old_particles:
            plt.scatter(old_part.x, old_part.y, s=markersize, color="yellow", alpha=0.2)
            # orientation
            arrow = plt.Arrow(old_part.x, old_part.y, scale*cos(old_part.orientation), scale*sin(old_part.orientation),width=width, alpha=0.2, facecolor='yellow', edgecolor='black')
            plt.gca().add_patch(arrow)
        # draw resampled particles
        for part in self.particles:
            # particle
            plt.scatter(part.x, part.y, s=markersize, color="green", alpha=0.2)
            # particle's orientation
            arrow = plt.Arrow(part.x, part.y, scale*cos(part.orientation), scale*sin(part.orientation), width=width,alpha=0.2, facecolor='green', edgecolor='black')
            plt.gca().add_patch(arrow)
        # draw landmarks
        for lm in ParticleFilter.WORLD.landmarks:
            plt.scatter(lm[0], lm[1], s=markersize*3, color="red")
        
        # draw true robot position & orientation
        plt.scatter(robot.trueX, robot.trueY, s=markersize*6, color="black", marker="s", label="Truth")
        arrow = plt.Arrow(robot.trueX, robot.trueY, scale*cos(robot.trueOrientation), scale*sin(robot.trueOrientation),width=width, facecolor='black', edgecolor='black')
        plt.gca().add_patch(arrow)
        # draw robot position & orientation according to movement
        plt.scatter(robot.x, robot.y, s=markersize*6, color="blue", marker="v", label="Estimate")
        arrow = plt.Arrow(robot.x, robot.y, scale*cos(robot.orientation), scale*sin(robot.orientation),width=width, facecolor='blue', edgecolor='blue')
        plt.gca().add_patch(arrow)
        # draw estimated robot position & orientation
        x, y, orientation = estimated_position
        plt.scatter(x, y, s=markersize*6, color="cyan", marker="v", label="Estimate")
        arrow = plt.Arrow(x, y, scale*cos(orientation), scale*sin(orientation),width=width, facecolor='cyan', edgecolor='cyan')
        plt.gca().add_patch(arrow)
        # save figure
        print("Save figure...")
        plt.savefig("data/figure_" + str(step) + ".png")
        plt.savefig("data/static.png")
        plt.close()

    """ Particle class for each particle in the particle filter """
    class Particle:
        def __init__(self):
            self.x = random.uniform(-ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)
            self.y = random.uniform(-ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)
            self.orientation = random.uniform(-1.0, 1.0) * pi

            # noise for the sensing and moving process
            self.forward_noise = 0.1
            self.turn_noise = 0.1
            self.sense_noise = 0.2

        def prettyPrint(self):
            print("Particle position: (%.2f, %.2f) \t Orientation: %.2f"%(self.x, self.y, self.orientation))

        def setPosition(self, new_x, new_y, new_orientation):
            # check values
            if (abs(new_x) > ParticleFilter.WORLD.size/2):
                raise ValueError("X out of bounds")
            if (abs(new_y) > ParticleFilter.WORLD.size/2):
                raise ValueError("Y out of bounds")
            if (abs(new_orientation) > pi):
                raise ValueError("Orientation not valid")
            # assign values
            self.x = float(new_x)
            self.y = float(new_y)
            self.orientation = float(new_orientation)

        def setNoise(self, new_forward_noise, new_turn_noise, new_sense_noise):
            # forward movement
            self.forward_noise = new_forward_noise
            # for turning
            self.turn_noise = new_turn_noise
            # for the sensor
            self.sense_noise = new_sense_noise

        def sense(self):
            allDistances = []
            # for each landmark, get the distance and add some noise
            for lm in ParticleFilter.WORLD.landmarks:
                distance = getDistance( (self.x, self.y), lm )
                distance += random.gauss(0.0, self.sense_noise)
                allDistances.append(distance)
            return allDistances

        def move(self, turn, forward):
            if (forward < 0):
                raise ValueError("Only forward kinematics allowed!")
            
            # orientation plus noise
            orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
            # has to be within range [-pi, pi]
            orientation = (orientation + pi) % (2 * pi) - pi

            # moving forward plus noise
            distance = float(forward) + random.gauss(0.0, self.forward_noise)
            x = self.x + (cos(orientation) * distance)
            y = self.y + (sin(orientation) * distance)
            # check boundaries
            x = limit(x, -ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)
            y = limit(y, -ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)

            # create new particle
            newParticle = ParticleFilter.Particle()
            newParticle.setPosition(x, y, orientation)
            newParticle.setNoise(self.forward_noise, self.turn_noise, self.sense_noise)
            return newParticle
        
        def getProbOfMeasurement(self, measurement):
            # calculate the measurement probability (how likely is this measurement)
            probability = 1.0
            for idx, lm in enumerate(ParticleFilter.WORLD.landmarks):
                distance = getDistance((self.x, self.y), lm)
                probability *= getGaussianP(distance, self.sense_noise, measurement[idx])
            return probability