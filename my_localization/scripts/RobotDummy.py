from math import *
from util import *
import random

class RobotDummy:
    def __init__(self, world):
        self.world = world
        self.x = random.uniform(-self.world.size/2, self.world.size/2)
        self.y = random.uniform(-self.world.size/2, self.world.size/2)
        self.orientation = random.uniform(-1.0, 1.0) * pi

        self.trueX = self.x
        self.trueY = self.y
        self.trueOrientation = self.orientation
        # noise for the sensing and moving process
        self.forward_noise = 0.1
        self.turn_noise = 0.1
        self.sense_noise = 0.2

    def prettyPrint(self):
        print("Particle position: (%.2f, %.2f) \t Orientation: %.2f"%(self.x, self.y, self.orientation))

    def setPosition(self, new_x, new_y, new_orientation):
        # check values
        if (abs(new_x) > self.world.size/2):
            raise ValueError("X out of bounds")
        if (abs(new_y) > self.world.size/2):
            raise ValueError("Y out of bounds")
        if (abs(new_orientation) > pi):
            raise ValueError("Orientation not valid")
        # assign values
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
        self.trueX = self.x
        self.trueY = self.y
        self.trueOrientation = self.orientation

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
        for lm in self.world.landmarks:
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
        x = limit(x, -self.world.size/2, self.world.size/2)
        y = limit(y, -self.world.size/2, self.world.size/2)

        # create new particle
        newParticle = RobotDummy(self.world)
        newParticle.setPosition(x, y, orientation)
        newParticle.setNoise(self.forward_noise, self.turn_noise, self.sense_noise)
        return newParticle
    
    def getProbOfMeasurement(self, measurement):
        # calculate the measurement probability (how likely is this measurement)
        probability = 1.0
        for idx, lm in enumerate(self.world.landmarks):
            distance = getDistance((self.x, self.y), lm)
            probability *= getGaussianP(distance, self.sense_noise, measurement[idx])
        return probability