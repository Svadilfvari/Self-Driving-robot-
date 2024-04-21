#!/usr/bin/env python


import rospy 
from sensor_msgs.msg import Imu

import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


# Create 

class LineFollowingNode:
    """A brief description of what the class does.

    This class is used to create an instance of it that would 
    manage the line Following strategy 
    """

    def __init__(self):

        """A brief description of what the function does.

        Constructor of the class, initiates the nodes and 
        the appropriate members for the control  strategy
        
        """
        # Initiates the Node
        rospy.init_node('line_following_node')

        # Speed publisher
        self.cmdVelPublisher=rospy.Publisher("/cmd_vel",Twist, queue_size=10)
        # Subscriber to the lanesCentroids topic that contains the centroids of the lanes
        self.imageSubscriber=rospy.Subscriber("/lanesCentroids",Float32MultiArray,self.lineFollowingCallback)
        
        # Subscriber to the obstacleDetected topic
        self.obstacleDetectionSubscriber=rospy.Subscriber("/meanDistancesLaser",Float32MultiArray,self.obstacleDetectionCallback)

        self.orientationSubscriber=rospy.Subscriber("/imu",Imu,self.orientationCallback)
        self.orientation=0
        self.targetOrientation=-.7

############ Variables for the obstacle avoidance strategy ############       
        # In this case we use the means of the distances provided by the Lidar
        # It helps reduce false positives due to noise or temporary obstructions
        self.frontLidar=0
        self.rightLidar=0
        self.leftLidar=0
        # Initialise the variable used to verify if an obstacle has been detected
        self.isObstacleDetectedInFront= False
        self.isObstacleDetectedAtRight=False
        self.isObstacleDetectedAtLeft=False
        # Distance detection threshold 
        self.thresholdDistance =.4

############ Variables for the line following strategy ############
        #Initializing the x coordinate of the centroid to zero 
        self.cX=0
        #Setting the target coordinate to zero 
        self.targetPixelCoordinate=0
        #Setting the variables used by the PID to zero 
        self.previousErrorLane = 0
        self.errorSumLane = 0
        self.timeStepLane = 0
        #Setting the gains of the PID controller
        self.proportionalGainControllerLane = 3.5 #.007 #rospy.get_param('/lanesCentroids/proportionalGainController', 0.05)
        self.integralGainControllerLane = 0.00000275  #rospy.get_param('/lanesCentroids/integralGainController', 0.0175)
        self.derivativeGainControllerLane = .09 #.0035#rospy.get_param('/lanesCentroids/derivativeGainController', 0.035)
        
        

############ Variables for the tunnel crossing strategy ############
         
        
        #Setting the target coordinate to zero 
        self.centerDistanceTunnel=0.5
        #Setting the variables used by the PID to zero 
        self.previousErrorTunnel = 0
        self.errorSumTunnel = 0
        self.timeStepTunnel = 0
        #Setting the gains of the PID controller
        self.proportionalGainControllerTunnel = 1.8#rospy.get_param('/lanesCentroids/proportionalGainController', 0.05)
        self.integralGainControllerTunnel = 0.000175 #rospy.get_param('/lanesCentroids/integralGainController', 0.0175)
        self.derivativeGainControllerTunnel = 0.0006#rospy.get_param('/lanesCentroids/derivativeGainController', 0.035)
                
    def orientationCallback(self,data):
        print(data.orientation.z)
        self.orientation=data.orientation.z

    def lineFollowingCallback(self,data):

        """A brief description of what the function does.
        Callback function to retrieve the image centroids x coordinate 
        """
        self.cX=data.data[0]   
        self.targetPixelCoordinate=data.data[1]   
    
    def obstacleDetectionCallback(self,data):
        """A brief description of what the function does.
        Callback function to retrieve the message indicating if an obstacle has been detected
        """
        self.frontLidar=data.data[0]
        self.rightLidar=data.data[2]
        self.leftLidar=data.data[3]

        # Checks if an obstacle has been detected in front
        if self.frontLidar is not None:
                if self.frontLidar<self.thresholdDistance :
                    self.isObstacleDetectedInFront=True
                #If an obstacle hasn't been detected it resets the flag    
                if self.frontLidar>self.thresholdDistance :
                    self.isObstacleDetectedInFront=False
                    
                  
        if self.rightLidar is not None:
            if self.rightLidar<self.thresholdDistance :
                self.isObstacleDetectedAtRight=True
            #If an obstacle hasn't been detected it resets the flag
            if self.rightLidar>self.thresholdDistance :

                self.isObstacleDetectedAtRight=False

        if self.leftLidar is not None:
                if self.leftLidar<self.thresholdDistance :
                    self.isObstacleDetectedAtLeft=True
                #If an obstacle hasn't been detected it resets the flag
                if self.leftLidar>self.thresholdDistance :
                    self.isObstacleDetectedAtLeft=False
         
    def obstacleAvoidanceStrategy(self):
        """A brief description of what the function does.
        A function to manage obstacle on the lane 
        """
        if self.isObstacleDetectedInFront:
            #Checks if the right direction is free of obstacles 
            # and that there's no line on the right (With marge of error of 45 pixels)
            if not self.isObstacleDetectedAtRight and self.cX<self.targetPixelCoordinate+45: 
                commandVelocity = Twist()
                commandVelocity.linear.x = 0.05
                commandVelocity.angular.z = -.1
                self.cmdVelPublisher.publish(commandVelocity)
            elif not self.isObstacleDetectedAtLeft and self.cX>self.targetPixelCoordinate+45: 
                commandVelocity = Twist()
                commandVelocity.linear.x = 0.05
                commandVelocity.angular.z = .1 
                self.cmdVelPublisher.publish(commandVelocity)

    def lineFollowingStrategy(self):
        """A brief description of what the function does.
        A function implementing a PID controller 
        to automatically follow the lines of the road
        """
        
        # Calculate error
        error = self.cX -self.targetPixelCoordinate  
        
        # Update error sum
        self.errorSumLane += error

        # Calculate derivative of error
        if self.timeStepLane > 0:
            dError = (error - self.previousErrorLane) / self.timeStepLane
        else:
            dError = 0

        # Calculate control output
        controlOutput = self.proportionalGainControllerLane * error +self.derivativeGainControllerLane * dError +self.integralGainControllerLane * self.errorSumLane  
        # Limit control output
        #controlOutput = np.clip(controlOutput, -self.maxAngularVelocity, self.maxAngularVelocity)

        # Create Twist message
        commandVelocity = Twist()
        commandVelocity.linear.x = 0.15-(np.abs(controlOutput)/1000*.5)  # Constant linear velocity
        commandVelocity.angular.z = controlOutput/100

        # Publish velocity command
        self.cmdVelPublisher.publish(commandVelocity)

        # Update last error and time
        self.previousErrorLane = error
        self.timeStepLane = 1 / 10  # Assuming control loop rate of 10 Hz
        #print("Pixel Error ", error )

    def isInTunnel(self):
        """A brief description of what the function does.
        Check if the robot is in the tunnel
        """
        tunnelWallsDistanceThreshold=.2
        # If the returned distances from the lateral lasers are lower than 
        # the threshold then the robot is within the walls of the tunnel

            #/!\ add a condition for the initial state 
                #add a way to compute how much do the distances decrease
        return all(0 < lidar < tunnelWallsDistanceThreshold for lidar in [self.rightLidar, self.leftLidar])
        #return  self.isObstacleDetectedAtRight and  self.isObstacleDetectedAtLeft


    def tunnelCrossingStrategy(self):
        """A brief description of what the function does.
        Implement a PID controller to ensure that the robot 
        automatically goes through the tunnel
        """

        # Calculate error
        error = (self.rightLidar+self.leftLidar)/2 -self.centerDistanceTunnel  

        # Update error sum
        self.errorSumTunnel += error

        # Calculate derivative of error
        if self.timeStepTunnel > 0:
            dError = (error - self.previousErrorTunnel) / self.timeStepTunnel
        else:
            dError = 0

        # Calculate control output
        controlOutputTunnel = self.proportionalGainControllerTunnel * error + self.integralGainControllerTunnel * self.errorSumTunnel + self.derivativeGainControllerTunnel * dError

        # Limit control output
        #controlOutput = np.clip(controlOutput, -self.maxAngularVelocity, self.maxAngularVelocity)

        # Create Twist message
        commandVelocity = Twist()
        commandVelocity.linear.x = 0.13-(np.abs(controlOutputTunnel)/1000*.5)  # Constant linear velocity
        commandVelocity.angular.z = controlOutputTunnel/50

        # Publish velocity command
        self.cmdVelPublisher.publish(commandVelocity)

        # Update last error and time
        self.previousErrorTunnel = error
        self.timeStepTunnel = 1 / 10  # Assuming control loop rate of 10 Hz
        #print("Pixel Error ", error )





    def controlStrategy(self):

        """A brief description of what the function does.
        Main function establishing the control strategy 
        """
        rate= rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.isObstacleDetectedInFront :
                rospy.loginfo("Obstacle detected, switching to obstacle avoidance mode")
                self.obstacleAvoidanceStrategy()
            #elif self.isInTunnel():
            #     rospy.loginfo("Tunnel detected, switching to tunnel crossing mode ")
                 #self.tunnelCrossingStrategy()
            else :
                rospy.loginfo("Lane detected, switching to lane follower mode")
                self.lineFollowingStrategy()
      

        
        rate.sleep()

            

           
     
if __name__ =='__main__':
    try:
        LineNode=LineFollowingNode()
        LineNode.controlStrategy()
    except rospy.ROSInterruptException:
        pass

            