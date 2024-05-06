#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ObstacleStopNode:

    def __init__(self):
        rospy.init_node('obstacle_stop')

        self.thresholdDistance = rospy.get_param("thereshold_distance", .001)
        self.obstacleDistance=None

        rospy.Subscriber("/meanDistancesLaser",Float32MultiArray,self.obstacleDistanceCallback)
        # Add a subscription to consider the fact that the robot
        # will always stay stil 
        self.cmdVelPublisher=rospy.Publisher("/cmd_vel",Twist, queue_size=10)
        
    def obstacleDistanceCallback(self,data):
        self.obstacleDistance=data.data[0]

    def stopWhenObstacle(self):
        rate= rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.obstacleDistance is not None:
                if (self.obstacleDistance)<(self.thresholdDistance) :
                    #The constructor by default sets the speeds to 0 
                    #print("obstacle detected")
                    stopVel=Twist()
                    self.cmdVelPublisher.publish(stopVel)
            rate.sleep()

if __name__ =='__main__':
    try:
        node=ObstacleStopNode()
        node.stopWhenObstacle()
    except rospy.ROSInterruptException:
        pass
