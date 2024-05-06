#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import math

#/!\/!\/!\/!\/!\Doubt on linear velocity of the tunnel .06 and .07

# Create the class 

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
        self.cmdVelPublisher = rospy.Publisher("/cmd_vel",Twist, tcp_nodelay = False, queue_size=1)
        # Subscriber to the lanesCentroids topic that contains the centroids of the lanes
        
        self.imageSubscriber = rospy.Subscriber("/lanesCentroids", Float32MultiArray, self.lineFollowingCallback)
        # Subscriber to the meanDistancesLaser topic
        self.obstacleDetectionSubscriber = rospy.Subscriber("/meanDistancesLaser", Float32MultiArray, self.obstacleDetectionCallback)
        # Subscriber to the imu topic to retrieve the z orientation of the robot used in the obstacle avoidance part 
        self.orientationSubscriber = rospy.Subscriber("/imu", Imu, self.orientationCallback)
        # Subscriber to the odom topic to retrieve the x and y coordinates of the robot used in the obstacle avoidance part
        # self.odometrySubscriber = rospy.Subscriber("/odom", Odometry, self.odometryCallback)

        # Parameters to store the position of the robot
        self.xPosition=0
        self.yPosition=0
        
        # Coordinates indicating the start of obstacle path
        self.xObstaclePathStarts=1.42
        self.yObstaclePathStarts=.25

        # Coordinates indicating the end of the obstacle path
        self.xObstaclePathEnds=1.7
        self.yObstaclePathEnds=-1.56
        
        # the current z orientation of the robot and the targeted one
        self.orientation=0
        self.targetOrientation=-.7

############ Variables for the obstacle avoidance strategy ############       
        # In this case we use the means of the distances provided by the Lidar
        # It helps reduce false positives due to noise or temporary obstructions
        self.frontLidar = 0
        self.rightLidar_max = 0
        self.leftLidar_max = 0
        self.rightLidar_min = 0
        self.leftLidar_min = 0

        # Initialize obstacle detection variables
        self.isObstacleDetectedAtRight = False
        self.isObstacleDetectedAtLeft = False

        # Initialize orietnation
        self.orientation_start  = 78.77#175
        self.orientation = 0
        self.rotation = 0

############ Variables for the line following strategy ############
        #Initializing the x coordinate of the centroid to zero 
        self.cX = 0
        #Setting the target coordinate to zero 
        self.targetPixelCoordinate = 0
        #Setting the variables used by the PID to zero 
        self.previousErrorLane = 0
        self.errorSumLane = 0
        self.timeStepLane = 0
        #Setting the gains of the PID controller
        self.proportionalGainControllerLane =1.5#1.3#3.5#3.5 #.007 #rospy.get_param('/lanesCentroids/proportionalGainController', 0.05)
        self.integralGainControllerLane = 0.000275  #rospy.get_param('/lanesCentroids/integralGainController', 0.0175)
 
        self.derivativeGainControllerLane = .1#.09 #.0035#rospy.get_param('/lanesCentroids/derivativeGainController', 0.035)

############ Variables for the tunnel strategy ############
        # Initialize the variables used for the PID controller
        self.previousErrorTunnel = 0
        self.timeStepTunnel = 0
        self.previousTime = rospy.Time.now()
        self.factor = 5  # 3 for discrete

        # Initialize the Flag for the isinTunnel function
        self.FLAGTunnel = False   

############ PID Variables for the tunnel crossing strategy ############
        
        #Setting the variables used by the PID to zero 
        self.previousErrorTunnel = 0
        self.timeStepTunnel = 0

        #Setting the gains of the PID controller
        self.proportionalGainControllerTunnel = 5
        self.derivativeGainControllerTunnel = 1
############ Challenges' transition management ############
        # Initiates a "Obstacle challenge starts" flag to False (0 in this case since the node subscribes to the /lanesCentroids topics that shares Float32MultiArray typed messages) 
        self.obstacleAvoidanceStarts=0
        # Initiates a "Tunnel challenge starts" flag to False (0 in this case since the node subscribes to the /lanesCentroids topics that shares Float32MultiArray typed messages) 
        self.tunnelCrossingStarts=0

############ List of all callback functions ############

    def lineFollowingCallback(self, data):
        """A brief description of what the function does.
        Callback function to retrieve the image centroids x coordinate 
        """
        self.cX = data.data[0]   
        self.targetPixelCoordinate = data.data[1]   
        self.obstacleAvoidanceStarts = data.data[2]
        self.tunnelCrossingStarts = data.data[3]
    
    def obstacleDetectionCallback(self, data):
        """A brief description of what the function does.
        Callback function to obtain the mean distances and
        indicate if an obstacle has been detected in front.
        """
        # Get the distances
        self.left_frontLidar = data.data[0]
        self.right_frontLidar = data.data[1]
        self.leftLidar_max = data.data[3]
        self.rightLidar_max = data.data[4]
        self.leftLidar_min = data.data[5]
        self.rightLidar_min = data.data[6]


        # Raise a Flag if an obstacle has been detected in either the left or right front
        self.isObstacleDetectedAtLeft = False
        if not np.isnan(self.left_frontLidar):
            self.isObstacleDetectedAtLeft = True

        self.isObstacleDetectedAtRight = False                    
        if not np.isnan(self.right_frontLidar):
            self.isObstacleDetectedAtRight = True

    def odometryCallback(self, data):
        """A brief description of what the function does.
        Callback function to retrieve the coordinate of the robot
        """
        self.xPosition=data.pose.pose.position.x
        self.yPosition=data.pose.pose.position.y

    def orientationCallback(self, data):
        """A brief description of what the function does.
        Callback function to retrieve the z orientation of the robot
        """
        self.orientation = self.quaternion_to_yaw (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

############ Other functions ############

    def isInTunnel(self):
        ''' A brief description of what the function does.
        Callback function to retrieve the message indicating if an obstacle has been detected
        '''
        # Raise a Flag if an obstacle if we are in the tunnel
        tunnelWallsDistanceThreshold = .2

        if self.rightLidar_max < tunnelWallsDistanceThreshold and self.leftLidar_max < tunnelWallsDistanceThreshold:
            self.FLAGTunnel = True

        if self.rightLidar_max > tunnelWallsDistanceThreshold and self.leftLidar_max > tunnelWallsDistanceThreshold:
            self.FLAGTunnel = False

        return self.FLAGTunnel
    
    def quaternion_to_yaw (self, x,y,z,w):
        ''' A brief description of what the function does.
        This function converts quaternionic orientation to yaw angle.
        
        :param: x (float): the x component of the quaternion.
        :param: y (float): the y component of the quaternion.
        :param: z (float): the z component of the quaternion.
        :param: w (float): the w component of the quaternion.

        :returns: yaw (float): the yaw angle in radians.
        '''

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        # Obtain yaw in radians
        yaw_rad = math.atan2(siny_cosp, cosy_cosp) 

        # Convert to degrees
        yaw_deg = yaw_rad * 180/math.pi

        # Map from [-180,180] to [0,360] angles
        if yaw_deg < 0:
            yaw_deg += 360
        return yaw_deg


############ List of all strategy functions ############

    def lineFollowingStrategy(self):

        """A brief description of what the function does.
        A function implementing a PID controller to automatically follow the lines of the road
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
        controlOutput = self.proportionalGainControllerLane * error + self.derivativeGainControllerLane * dError + self.integralGainControllerLane * self.errorSumLane  
        # Limit control output
        #controlOutput = np.clip(controlOutput, -self.maxAngularVelocity, self.maxAngularVelocity)

        # Create Twist message
        commandVelocity = Twist()
        # commandVelocity.linear.x = 0.15-(np.abs(controlOutput)/1000*.5)  # Constant linear velocity
        # commandVelocity.angular.z = controlOutput/100

        
        ########### Code for real robot starts
        commandVelocity.linear.x = abs(.09-(np.abs(controlOutput)/1500*.5)) #0.15-(np.abs(controlOutput)/2000*.5)  # Constant linear velocity
        commandVelocity.angular.z = controlOutput/450#500 #100
        
        # commandVelocity.angular.z = -commandVelocity.angular.z
        ########### Code for real robot ends
        # Publish velocity command
        self.cmdVelPublisher.publish(commandVelocity)
       

        # Update last error and time
        self.previousErrorLane = error
        self.timeStepLane = 1 / 20  # Assuming control loop rate of 10 Hz
        #print("Pixel Error ", error )
         
    def obstacleAvoidanceStrategy(self):
        """A brief description of what the function does.
        A function to manage obstacle on the lane 
        """
          # Initialize Twist message for command velocity
        commandVelocity = Twist()

        # If obstacle detected on the left, adjust velocities
        if self.isObstacleDetectedAtLeft:
            distance = self.left_frontLidar
            commandVelocity.angular.z = - (0.7 - distance)  # 0.5
            commandVelocity.linear.x = (0.1) * (0.7 - distance)  # 0.5
            self.rotation = -1
        # If obstacle detected on the right, adjust velocities
        elif self.isObstacleDetectedAtRight:
            distance = self.right_frontLidar
            commandVelocity.angular.z = (0.7 - distance)  # 0.5
            commandVelocity.linear.x = (0.1) * (0.7 - distance)
            self.rotation = +1
        else:
            # Continue on current trajectory
            commandVelocity.angular.z = - self.rotation * 0.5
            commandVelocity.linear.x = 0.02
            # If orientation change is small, stop angular velocity and increase linear velocity
            if abs(self.orientation - self.orientation_start) < 5:
                commandVelocity.angular.z = 0
                commandVelocity.linear.x = 0.1

        # Publish the command velocity
        self.cmdVelPublisher.publish(commandVelocity)

    def tunnelCrossingStrategy(self):
        """A brief description of what the function does.
        A function to manage the crossing of the tunnel 
        """

        # Calculate the error: only proportional and derivative term

        # Proportional term
        error = min(self.rightLidar_min - self.leftLidar_min, 0)
        print(self.rightLidar_min , self.leftLidar_min)

        # Derivative term
        dError = 0
        if self.timeStepTunnel > 0:
            # Calculate time variation 
            currentTime = rospy.Time.now()
            timeDifference = (currentTime - self.previousTime).to_sec()
            
            # Obtain derivative of the error
            if timeDifference != 0.0: # sanity check             
                dError = (error - self.previousErrorTunnel)  / timeDifference  

            # Update time and error
            self.previousTime = currentTime
            self.previousErrorTunnel = error
        
        # Initialize Twist message
        commandVelocity = Twist()

        # Set velocities 
        commandVelocity.angular.z = - self.proportionalGainControllerTunnel * error - self.derivativeGainControllerTunnel * dError
        #print(commandVelocity.angular.z)
        commandVelocity.linear.x = max(0.06 - abs(error/10), 0)           
        
        # Publish velocity command
        self.cmdVelPublisher.publish(commandVelocity)
        
        self.timeStepTunnel = 1
        
############ Control strategy function ############

    def controlStrategy(self):

        """A brief description of what the function does.
        Main function establishing the control strategy 
        """
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            # If we are in the tunnel, proceed with the tunnel strategy
            # When the robots detects the blue strip of the tunnel and the obstacle Avoidance has already started, it means 
            # that the robot have already crossed the obstacle field indicated by the obstacleAvoidanceStarts flag 
            if self.tunnelCrossingStarts and self.obstacleAvoidanceStarts:
                # As long as the robot isn't inside the tunnel it moves forward 
                while(not self.isInTunnel()):
                    
                    commandVelocity = Twist()
                    commandVelocity.linear.x = .1          
                    # Publish velocity command
                    self.cmdVelPublisher.publish(commandVelocity)
                
                # When the code reaches this part, it means the robot entered the tunnel
                rospy.loginfo("Tunnel detected, switching to tunnel mode. ")
                self.tunnelCrossingStrategy()
            
            # else: # otherwise, distinguish between the obstacle avoidance and line following strategies.
            #     if self.isObstacleDetectedAtLeft:
            #         rospy.loginfo("Obstacle detected to the left, switching to obstacle avoidance mode. ")
            #         self.obstacleAvoidanceStrategy()
            #     elif self.isObstacleDetectedAtRight:
            #         rospy.loginfo("Obstacle detected to the right, switching to obstacle avoidance mode")
            #     else:
            #         self.lineFollowingStrategy()
            #         rospy.loginfo("Lane detected, switching to lane follower mode")
            
            # When the robot detects the  blue strip of the obstacle field and hasn't detected the tunnel it proceeds with the obstacle strategy 
            elif self.obstacleAvoidanceStarts  : # otherwise, distinguish between the obstacle avoidance and line following strategies.
                if self.isObstacleDetectedAtLeft:
                    rospy.loginfo("Obstacle detected to the left, switching to obstacle avoidance mode. ")
                    self.obstacleAvoidanceStrategy()
                elif self.isObstacleDetectedAtRight:
                    rospy.loginfo("Obstacle detected to the right, switching to obstacle avoidance mode")
                    self.obstacleAvoidanceStrategy()
                # When neither of the aforementioned conditions are satisfied it proceeds with the following of the lane 
                else:
                    rospy.loginfo("No obstacle detected, following the path")
                    self.obstacleAvoidanceStrategy()
            # path following
            else:
                rospy.loginfo("Lane detected, following the path")
                self.lineFollowingStrategy()
            rate.sleep()


############ Main function ############

if __name__ =='__main__':
    try:
        LineNode=LineFollowingNode()
        LineNode.controlStrategy()
    except rospy.ROSInterruptException:
        pass

            