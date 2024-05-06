#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
class LaserControlStrategy:

    def __init__(self):
        #forward
        self.front_mean=0
        #Right
        self.right_mean=0
        #back 
        self.back_mean=0
        #left
        self.left_mean=0

        rospy.init_node('lds_distance',anonymous=True)

        self.pubMeans=rospy.Publisher("/meanDistancesLaser", Float32MultiArray, queue_size=10)

        rospy.Subscriber('/scan',LaserScan, self.laserCallback)

    def laserCallback(self,data):

        maxDistance=10.
        
        
        
        filteredRanges=[min(maxDistance,range) for range in data.ranges]
        

        # #forward
        # forwardArray=filteredRanges[0:89]
        # #Right
        # rightArray=filteredRanges[90:179]
        # #back 
        # backArray=filteredRanges[180:269]
        # #left
        # leftArray=filteredRanges[270:359]
        
        #forward
        forwardArray=filteredRanges[0:20]+filteredRanges[-20:]
        #back
        backArray=filteredRanges[160:200]#+filteredRanges[180:200]
        #right
        rightArray=filteredRanges[270:290]
        #left
        leftArray=filteredRanges[90:110]
        
        

        #forward
        self.front_mean=np.mean(forwardArray)
        #back 
        self.back_mean=np.mean(backArray)
        #Right
        self.right_mean=np.mean(rightArray)
        #left
        self.left_mean=np.mean(leftArray)

   
    
    def laserDistanceStrategyPublisher(self):
        
        

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            #print("Front mean ",self.front_mean,"Back mean ",self.back_mean,"Right mean ",self.right_mean,"Left mean ",self.left_mean," \n")
            meanArray = [self.front_mean, self.back_mean, self.right_mean, self.left_mean]
            self.pubMeans.publish(Float32MultiArray(data=meanArray))
            rate.sleep()
        

       



if __name__ == '__main__':

    try:    
        node = LaserControlStrategy()
        node.laserDistanceStrategyPublisher()

    except rospy.ROSInterruptException:
        pass