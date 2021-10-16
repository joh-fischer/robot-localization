#!/usr/bin/env python
# @Author:  Johannes S. Fischer

""" some helper functions for the mobile robotics project """
from math import *
from geometry_msgs.msg import Twist
import numpy as np

# function to compute the distance between two points in 2D space
def getDistance(pos1, pos2):
    # @param:
    # pos1, pos2    either tupel or array containing x and y coordinate
    x1, y1 = pos1
    x2, y2 = pos2
    return sqrt( (x1 - x2)**2 + (y1 - y2)**2 )

# get probability of x, given mu and sigma
def getGaussianP(mu, sigma, x):
    # see https://en.wikipedia.org/wiki/Gaussian_function
    variance = float(sigma)**2
    denominator = sqrt(2*pi*variance)
    numerator = exp( - (float(x) - float(mu))**2 / (2*variance))
    return numerator / denominator

# limit a number
def limit(x, min_x, max_x):
    return max( min(max_x, x), min_x)

# adapted from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# mean of circular quantities according to https://en.wikipedia.org/wiki/Mean_of_circular_quantities
def mean_circular_quantities(alpha_arr, weights):
    sum_sin = 0
    sum_cos = 0
    for i in range(len(alpha_arr)):
        alpha = alpha_arr[i]
        weight = weights[i]
        sum_sin += weight * sin(alpha)
        sum_cos += weight * cos(alpha)
    return atan2(sum_sin, sum_cos)

# normalize array (e.g. weights)
def normalize(arr):
    sum_arr = np.sum(arr)
    normalized_arr = [i/sum_arr for i in arr]
    return normalized_arr
    
# class for the velocity message
class VelMsg:
    def __init__(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.msg = vel_msg

    def setForward(self, speed):
        self.msg.linear.x = speed
    
    def setTurn(self, speed):
        self.msg.angular.z = speed