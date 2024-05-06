#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LaserControlStrategy:

    def __init__(self):

        # Initialize the mean distances

        # Left forward
        self.left_forward_mean = 0
        # Right forward
        self.right_forward_mean = 0
        # Forward
        self.forward_mean = 0
        # Left max
        self.left_mean_max = 0
        # Right max
        self.right_mean_max = 0
        # Left min
        self.left_mean_min= 0
        # Right min
        self.right_mean_min = 0

        # Initialize the node, publisher and subscriber
        rospy.init_node('lds_distance', anonymous = True)
        self.pubMeans = rospy.Publisher("/meanDistancesLaser", Float32MultiArray, queue_size = 2)
        self.subScan = rospy.Subscriber('/scan', LaserScan, self.laserCallback)

    def laserCallback(self, data):

        # Minimum distance detectable
        minDistance = 0.01
        # Maximum distance detectable
        maxDistance = 3.5

        # Angle aperture for the lidar distances in obstacle avoidance strategy
        delta_forward = 40
        # Angle aperture for the lidar distances in tunnel strategy
        delta = 20  # 5 for simulation, 5 for discrete controlling

        # Treshold distance for obstacle detection
        obstacle_threshold = 0.30
        
        # filteredRanges = [max(minDistance, ranges) for ranges in data.ranges]
 
        # Obtain forward distances: out-of-range values are mapped to maxDistance
        forwardArray = data.ranges[0:delta] + data.ranges[-delta:]

        # Obtain forward distances for obstacle detection: out-of-range values are mapped to maxDistance
        left_forwardArray = np.array(data.ranges[0:delta_forward])
        left_forwardArray[np.where(left_forwardArray < minDistance)] = maxDistance
        left_forwardArray = left_forwardArray[np.where(left_forwardArray < obstacle_threshold)]

        right_forwardArray = np.array(data.ranges[-delta_forward:])
        right_forwardArray[np.where(right_forwardArray < minDistance)] = maxDistance
        right_forwardArray = right_forwardArray[np.where(right_forwardArray < obstacle_threshold)]

        # Obtain left and right distances
        leftArray = data.ranges[(90 - 2*delta):(90)]
        leftArray = np.array (leftArray)

        rightArray = data.ranges[(270):(270 + 2*delta)]
        rightArray = np.array (rightArray)

        # Map out-of-range values with maxDistance and minDistance
        leftArray_max = np.copy(leftArray)
        leftArray_min = np.copy(leftArray)
        leftArray_max[np.where(leftArray < minDistance)] = maxDistance
        leftArray_min[np.where(leftArray < minDistance)] = minDistance

        rightArray_max = np.copy(rightArray)
        rightArray_min = np.copy(rightArray)
        rightArray_max[np.where(rightArray < minDistance)] = maxDistance
        rightArray_min[np.where(rightArray < minDistance)] = minDistance


        # Calculate the averages
        # Left forward
        self.left_forward_mean = np.mean(left_forwardArray)
        # Right forward 
        self.right_forward_mean = np.mean(right_forwardArray)
        # Forward
        self.forward_mean = np.mean(forwardArray)
        # Left max mapped
        self.left_mean_max = np.mean(leftArray_max)
        # Right max mapped
        self.right_mean_max = np.mean(rightArray_max)
        # Left max mapped
        self.left_mean_min = np.mean(leftArray_min)
        # Right max mapped
        self.right_mean_min = np.mean(rightArray_min)

  
    def laserDistanceStrategyPublisher(self):
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            meanArray = [self.left_forward_mean, self.right_forward_mean, self.forward_mean, self.left_mean_max, self.right_mean_max, self.left_mean_min, self.right_mean_min]
            self.pubMeans.publish(Float32MultiArray(data=meanArray))
            rate.sleep()

if __name__ == '__main__':

    try:    
        node = LaserControlStrategy()
        node.laserDistanceStrategyPublisher()

    except rospy.ROSInterruptException:
        pass