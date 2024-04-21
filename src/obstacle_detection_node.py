#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class ObstacleDetectionNode:

    def __init__(self):
        rospy.init_node('obstacle_detection_node')

        self.thresholdDistance=rospy.get_param("thereshold_distance", .001)
        self.obstacleDistance=None

        rospy.Subscriber("/meanDistancesLaser",Float32MultiArray,self.obstacleDistanceCallback)
        # Add a subscription to consider the fact that the robot
        # will always stay stil 
        self.obstacleDetectionPublisher=rospy.Publisher("/obstacle_detection",Bool, queue_size=10)
        
    def obstacleDistanceCallback(self,data):
        # retrieve the front laser data
        self.obstacleDistance=data.data[0]

    def obstacleDetection(self):
        rate= rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.obstacleDistance is not None:
                if self.obstacleDistance<self.thresholdDistance :
                    
                    #print("obstacle detected")
                    obstacleDetected=True
                    self.obstacleDetectionPublisher.publish(obstacleDetected)
        rate.sleep()

if __name__ =='__main__':
    try:
        node=ObstacleDetectionNode()
        node.obstacleDetection()
    except rospy.ROSInterruptException:
        pass
