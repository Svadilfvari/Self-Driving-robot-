#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from PIL import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# Create 

class ImageProcessingNode:
    """A brief description of what the class does.

    This class is used to create an instance of it that would 
    process images for the line following strategy and publishes 
    the moments thereafter 
    """

    def __init__(self):

        """A brief description of what the function does.

        Constructor of the class, initiates the nodes and 
        the appropriate members for the image processing 
        
        """
        # Initiates the Node
        rospy.init_node('image_processing_node',anonymous=True)
        self.cY=0
        self.cX=0
        # Assigns the member "bridge" with an instance of "CvBridge"
        # In order to interface ROS and OpenCV (convert the sensor_msgs/Image into an OpenCV class)

        self.bridgeROStoCV=CvBridge()
        # Initiating the publisher that shares the x coordinate of the centroids of the lane 
        # (i.e in control terms, the centroids are the current state)
        # and shares the X coordinate corresponding to the middle of the image
        # (i.e in control terms, the middle of the image is the target state)
        self.pubCentroids=rospy.Publisher("/lanesCentroids",Float32MultiArray, queue_size=1)
        
        #Simulation Subscriber 
        #self.imageSubscriber=rospy.Subscriber("/camera/image",Image,self.imageCb)
        
        # Real World Subscriber 
        self.imageSubscriber=rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.imageCb)
        
        # Setting the target coordinate (i.e The set point) to zero 
        self.targetPixelCoordinate=0
        rospy.loginfo("Image processing starts")
        #######Challenges' transition management#######
        # Initiates a "Obstacle challenge starts" flag to False (0 in this case to be able to publish it within the /lanesCentroids topics that shares Float32MultiArray typed messages) 
        self.obstacleAvoidanceStarts=0
        # Initiates a "Tunnel challenge starts" flag to False (0 in this case to be able to publish it within the /lanesCentroids topics that shares Float32MultiArray typed messages) 
        self.tunnelCrossingStarts=0
        
       
    def imageCb(self,data):
        """A brief description of what the function does.
        Callback function to process images
        """
           
        # First, we convert the image into a cv2 image
        # The following block handles the eventual exceptions
        try:
            ########### Code for real robot starts
            np_arr = np.fromstring(data.data, np.uint8)
            cvImage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            ########### Code For real robot ends
            
            #Simulation
            #cvImage= self.bridgeROStoCV.imgmsg_to_cv2(data,desired_encoding='bgr8')
        except CvBridgeError as e:
             print(e)
        
        # cv2.imshow("original Image",cvImage)
        # cv2.waitKey(1)  
        # cv2.namedWindow('original Image')
        # cv2.setMouseCallback('original Image', self.get_coordinate)
        
        # cv2.waitKey(1)
        
        # Image segmentation 
        # Idea : https://www.thinkautonomous.ai/blog/lane-detection/
        height,width,channels=cvImage.shape[0],cvImage.shape[1],cvImage.shape[2]
        #print(h,w)
        # # # Create binary mask where the pixel of the Region of Interest are set to white
        # # #using a polygon covering the lane
        mask = np.zeros_like(cvImage)
        match_mask_color = (255,) * channels    
        
        #polygon = np.array([[30,230],[90,160],[230,160],[width+50,width+50]]) 
        
        #polygon = np.array([[width+270,width],[h,250],[120,250],[-200,width]]) 
        polygon = np.array([[width//4,0],[width//4,height//5],[3*width//4,height//5],[3*width//4,0]]) 
        cv2.fillConvexPoly(mask, polygon, match_mask_color)
        
        # masked_image = cv2.bitwise_and(cvImage, mask)
        # cv2.imshow("Masked Image",masked_image)
        # cv2.waitKey(1)



        #imgblur = cv2.GaussianBlur(cvImage, (5,5), 3)
        
        imgblur=cv2.bilateralFilter(cvImage,20,75,75)
        # cv2.imshow("imgblur Image",imgblur)
        # cv2.waitKey(1)
        
        #imgblur=bm3d.bm3d(imgblur,sigma_psd=30/255,stage_arg=bm3d.BM3DStages.HARD_THRESHOLDING)
        #SInce the bm3DDer BM3D-Filter kann den Pixeltyp des Bildes ändern, 
        #insbesondere wenn der sigma_psd-Parameter verwendet wird. 
        #Dadurch kann es zu einer Typänderung des Bildes kommen
        
        imgblur = imgblur.astype(np.uint8)
        # cv2.imshow("blured Image",imgblur)
        # cv2.waitKey(1)

        ########### Code for real robot starts
        kernelOpening = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        opening = cv2.morphologyEx(imgblur,cv2.MORPH_OPEN,kernelOpening)
        kernelClosing = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernelClosing)

        # Brightness adjustment (using a gamma function)
        
        gammaAdjusted=self.adjust_gamma(closing, gamma=1.2)
        # cv2.imshow("Gamma Adjusted Image",gammaAdjusted)
        # cv2.namedWindow('Gamma Adjusted Image')
        # cv2.setMouseCallback('Gamma Adjusted Image', self.get_coordinate)
        # cv2.waitKey(1)

        # Pink print(gammaAdjusted[211,586])
        # Road 
        #print(gammaAdjusted[220,100])
        # print("red",gammaAdjusted[244,574])
        # print("green",gammaAdjusted[276,40])
        ########### Code for real robot ends

        # Convert BGR to HSV
        hsv = cv2.cvtColor(gammaAdjusted, cv2.COLOR_BGR2HSV)
        # cv2.imshow("hsv",hsv)
        # cv2.namedWindow('hsv')
        # cv2.setMouseCallback('hsv', self.get_coordinate)
        # cv2.waitKey(1)

        ########### Code for real robot starts

        lower_green = np.array([100,26,20])
        higher_green = np.array([166,255,255])
        mask_green = cv2.inRange(hsv, lower_green, higher_green)

 
        lower_red = np.array([50,47,60])#np.array([50,47,60])#np.array([50,47,60]) #np.array([134,30,0])
        higher_red = np.array([180,196,201])#np.array([179,255,255])
        mask_red = cv2.inRange(hsv, lower_red, higher_red)

        

###### Detecting the blue strip to enable transitions between challenges #######
       #######Lane following -> Obstacle avoidance transition's management ####### 
        lower_blue=np.array([100,100,64])#[162,100,64])
        higher_blue=np.array([180,100,255])
        obstacleMask_blue=cv2.inRange(hsv, lower_blue, higher_blue)
        # cv2.imshow("mask blue Image",mask_blue)
        # cv2.waitKey(1)
        # We only keep the bottom middle part of the image that contains
        # the blue strip
        #[[width//4,height],[width//4,],[3*width//4,5*height//6],[3*width//4,height]]) 
        obstacleMask_blue[0:5*height//6,:]=0
        # We crop it further to keep a significant part of the blue strip  
        obstacleMask_blue[:,0:width//4]=0
        obstacleMask_blue[:,3*width//4:width]=0
        



        # Apply the canny filter to detect edges
        t_lower = 50  # Lower Threshold 
        t_upper = 150  # Upper threshold 
        obstacleMask_blueEdges = cv2.Canny(obstacleMask_blue,t_lower,t_upper)
        # cv2.imshow("obstacleMask_blueEdges",obstacleMask_blueEdges)
        # cv2.namedWindow('obstacleMask_blueEdges')
        # cv2.waitKey(1)
        
        # Computes the moments of the obstacleMask_blueEdges to detect the blue strip 
        obstacleBlueStripMoments = cv2.moments(obstacleMask_blueEdges)
        
        # Checks if the blue strip has been detected
        # moments that are greater than 10000 mean that a "intense blue" has been detected
        if obstacleBlueStripMoments["m00"]>0:
            print(obstacleBlueStripMoments["m00"])
            cX = int(obstacleBlueStripMoments["m10"] / obstacleBlueStripMoments["m00"])
            cY = int(obstacleBlueStripMoments["m01"] / obstacleBlueStripMoments["m00"])
        # Sets a "Obstacle challenge starts" flag to true (1 in this case to be able to publish it within the /lanesCentroids topics that shares Float32MultiArray typed messages) 
            self.obstacleAvoidanceStarts=1
            # print(obstacleBlueStripMoments["m00"])
        
        else:
            cX=0
            cY=0
        

        cv2.circle(obstacleMask_blue, (cX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # draw a circle on the center of the image
        
        # cv2.imshow("modified mask blue Obstacle",obstacleMask_blue)
        # cv2.namedWindow('modified mask blue Obstacle')
        # cv2.waitKey(1)
        
    ###### Obstacle avoidance -> Tunnel crossing  transition's management ###### 
        
        # Will be used the same mask used for the obstacle avoidance's transition 
        tunnelMask_blue=cv2.inRange(hsv, lower_blue, higher_blue)
 
        # We only keep the top middle part of the image that contains
        # the blue strip
        tunnelMask_blue[height//5:height,:]=0

        # We crop it further to keep a significant part of the blue strip  
        tunnelMask_blue[:,0:width//4]=0
        tunnelMask_blue[:,3*width//4:width]=0
        
        # Apply the canny filter to detect edges
        t_lower = 50  # Lower Threshold 
        t_upper = 150  # Upper threshold 
        tunnelMask_blueEdges = cv2.Canny(tunnelMask_blue,t_lower,t_upper)

        # Computes the moments of the tunnelMask_blueEdges to detect the blue strip 
        tunnelBlueStripMoments = cv2.moments(tunnelMask_blueEdges)
        
        # Checks if the blue strip has been detected
        # moments that are greater than 3000 mean  that a "intense blue" has been detected
        if tunnelBlueStripMoments["m00"]>3000:
            cX = int(tunnelBlueStripMoments["m10"] / tunnelBlueStripMoments["m00"])
            cY = int(tunnelBlueStripMoments["m01"] / tunnelBlueStripMoments["m00"])
        # Sets a "tunnel challenge starts" flag to true (1 in this case to be able to publish it within the /lanesCentroids topics that shares Float32MultiArray typed messages) 
        # Checks if the the obstac    
            if self.obstacleAvoidanceStarts==1:
                self.tunnelCrossingStarts=1
        
        else:
            cX=0
            cY=0
        

        cv2.circle(tunnelMask_blue, (cX, cY), 20, (255,255,255), -1)  #mark the center of the contour
        # draw a circle on the center of the image
        
        # cv2.imshow("modified mask blue Tunnel",tunnelMask_blue)
        # cv2.namedWindow('modified mask blue Tunnel')
        # cv2.waitKey(1)



        ##Combine masks
        mask = cv2.bitwise_or(mask_green, mask_red)
        # cv2.imshow("mask green Image",mask_green)
        # cv2.waitKey(1)
        # cv2.imshow("mask  Image",mask)
        # cv2.waitKey(1)
        ########### Code for real robot ends


        ########### Code for Simulated robot starts

        # Define range of white color in HSV
        # lower_white = np.array([0, 0, 200])
        # upper_white = np.array([255, 30, 255])

        # # Threshold the HSV image to get only white colors
        # mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # # Define range of yellow color in HSV
        # lower_yellow = np.array([20, 100, 100])
        # upper_yellow = np.array([30, 255, 255])

        # # Threshold the HSV image to get only yellow colors
        # mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # # Combine masks
        # mask = cv2.bitwise_or(mask_white, mask_yellow)

        ########### Code for Simulated robot ends


        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cvImage, cvImage, mask=mask)

        # Convert to grayscale
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        
        # # Convert the image from RGB to HSV color space
        # hsv_image = cv2.cvtColor(imgblur, cv2.COLOR_BGR2HSV)

        # # Define range for red color in HSV
        # lower_red = np.array([0, 100, 100])
        # upper_red = np.array([10, 255, 255])
        # lower_red2 = np.array([160, 100, 100])
        # upper_red2 = np.array([180, 255, 255])

        # # Threshold the HSV image to get only red colors
        # red_mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        # red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        # red_mask = cv2.bitwise_not(red_mask1, red_mask2)

        # # Invert the mask (optional, depending on whether you want to keep or remove red objects)
        # # red_mask = cv2.bitwise_not(red_mask)

        # # Apply the mask to the original image
        # result = cv2.bitwise_and(hsv_image, hsv_image, mask=red_mask)

        # cv2.imshow("0 red Image",result)
        # cv2.waitKey(1)
       
        
        # Setting parameter values 

        t_lower = 50#int(max(0,(1.0-sigma)*v))#50  # Lower Threshold 
        t_upper = 150#int(min(255,(1.0+sigma)*v))#150  # Upper threshold 
        # cv2.imshow(" Image before edge detection",res)
        
        # cv2.waitKey(1)
        # Apply the canny filter to detect edges
        edges = cv2.Canny(mask_red,t_lower,t_upper)

        
 
        
        ########### Code For real robot starts
        # Selecting the areas of interest (in this cas the robot requires the bottom part of the screen)  
        # /!\ Note: A better idea is to use a dynamic ROI, that would changes depending on
        # whether or not a line has been detected 
        search_top = 3*height//8 #camera look directly beneath 
        search_bot = 7*height//8 
        # Setting the other regions to zero
        edges[0:search_top, 0:width] = 0
        edges[search_bot:height, 0:width ]= 0
        edges[:,width//6:5*width//6]=0

        # closing the edges 
        # kernelOpening = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        # edges = cv2.morphologyEx(edges,cv2.MORPH_OPEN,kernelOpening)
        # kernelClosing = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        # edges = cv2.morphologyEx(edges,cv2.MORPH_CLOSE,kernelClosing)

        ########### Code For real robot ends
        
        
        # Function used to retrieve the coordinates where the mouse clicks 
        # The coordinates are used for the creation of the polygon
        # cv2.imshow("modified Image",edges)
        # cv2.namedWindow('modified Image')
        # cv2.setMouseCallback('modified Image', self.get_coordinate)
        
        # cv2.waitKey(1)
        

        # Divide the image by half vertically
        # height, width = edges.shape
        # edges[0:height//2, 0:width]=0
        # Compute the moments

        
        #### Red line's centroid #####
        # Isolate the red line
        redEdges=edges.copy()
        redEdges[:,0:5*width//6]=0
        M = cv2.moments(redEdges)
        # parse returned data from "moments"
        if M["m00"]>0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            
            # Update the coordinates   
            redcX=cX
            self.cY=cY
            # 29,338, with red 463,323
            # 305 found by finding manually the mid of the image
            # this was done due to the introduced distortion by the camera
            self.targetPixelCoordinate=305 #328#66#1*width//2






            # draw the center of the shape on the image
            # cv2.circle(redEdges, (redcX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # # draw a circle on the center of the image
            # cv2.circle(redEdges, (self.targetPixelCoordinate, cY), 20, (150,150,255), -1)
            # # Display the image with the circles on it 
            # cv2.imshow("red edges Image",redEdges)
            # cv2.waitKey(1)

        # if the green line is not detected
        else:
            redcX=0
            
 
            self.targetPixelCoordinate=width #66#1*width//2




            # # draw the center of the shape on the image
            # cv2.circle(greenEdges, (cX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # # draw a circle on the center of the image
            # cv2.circle(greenEdges, (self.targetPixelCoordinate, cY), 20, (150,150,255), -1)
            # # Display the image with the circles on it 
            # cv2.imshow("green edges Image",greenEdges)
            # cv2.waitKey(1)
    

        #### green line's centroid #####
        # Isolate the green line
        greenEdges=edges.copy()
        greenEdges[:,1*width//2:width]=0


        M = cv2.moments(greenEdges)
        # parse returned data from "moments"
        if M["m00"]>0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            
            # Update the coordinates   
            greencX=cX
            self.cY=cY
            # 29,338, with red 463,323
            self.targetPixelCoordinate=305 #66#1*width//2

            # # draw the center of the shape on the image
            # cv2.circle(greenEdges, (greencX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # # draw a circle on the center of the image
            # cv2.circle(greenEdges, (self.targetPixelCoordinate, cY), 20, (150,150,255), -1)
            # # Display the image with the circles on it 
            # cv2.imshow("green edges Image",greenEdges)
            # cv2.waitKey(1)

        # if the green line is not detected
        else:
            greencX=0
            
            # 29,338, with red 463,323
            self.targetPixelCoordinate=0 #66#1*width//2




            # draw the center of the shape on the image
            # cv2.circle(greenEdges, (greencX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # # draw a circle on the center of the image
            # cv2.circle(greenEdges, (self.targetPixelCoordinate, cY), 20, (150,150,255), -1)
            # # Display the image with the circles on it 
            # cv2.imshow("green edges Image",greenEdges)
            # cv2.waitKey(1)
        # The resulting centroid is the mean of the two aforementioned centroids    
        
        self.cX=(greencX+redcX)//2

        
        # cv2.circle(edges, (self.cX, self.cY), 20, (255,255,255), -1)  #mark the center of the contour
        #     # draw a circle on the center of the image
        # cv2.circle(edges, (self.targetPixelCoordinate, self.cY), 20, (150,150,255), -1)
        # cv2.imshow("modified Image",edges)
        # cv2.namedWindow('modified Image')
        # cv2.waitKey(1)



    def get_coordinate(self,event, x, y,flags, param):
        """A brief description of what the function does.
        Function used to retrieve coordinates where the mouse clicks
        Those coordinates are used for the segmentation of the image
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Clicked coordinate: ({}, {})".format(x, y))
    
    def adjust_gamma(self,image, gamma=1.0):
        """A brief description of what the function does.
        Function used readjust the brightness of an image
        Used on the real world robot
        """
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)


    def publishMoments(self):
        """ This is the main function that will use all the methods 
            of the hereby class and publishes the moments
        """
        rate= rospy.Rate(20)
        
        while not rospy.is_shutdown():
            centroids= [self.cX,self.targetPixelCoordinate,self.obstacleAvoidanceStarts,self.tunnelCrossingStarts]
            self.pubCentroids.publish(Float32MultiArray(data=centroids))
            rate.sleep()    

if __name__ =='__main__':
    try:
        imageProcessing=ImageProcessingNode()
        imageProcessing.publishMoments()
    except rospy.ROSInterruptException:
        pass



