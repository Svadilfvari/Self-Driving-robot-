#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
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
        self.pubCentroids=rospy.Publisher("/lanesCentroids",Float32MultiArray, queue_size=10)
        #Simulation Subscriber 
        self.imageSubscriber=rospy.Subscriber("/camera/image",Image,self.imageCb)
        #Real World Subscriber 
        #self.imageSubscriber=rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.imageCb)
        
        #Setting the target coordinate (i.e The set point) to zero 
        self.targetPixelCoordinate=0
        rospy.loginfo("Image processing starts")
        
       
    def imageCb(self,data):
        """A brief description of what the function does.
        Callback function to process images
        """
           
        # First, we convert the image into a cv2 image
        # The following block handles the eventual exceptions error 
        try:
            #np_arr = np.fromstring(data.data, np.uint8)
            #cvImage = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #Simulation
            cvImage= self.bridgeROStoCV.imgmsg_to_cv2(data,desired_encoding='bgr8')
        except CvBridgeError as e:
             print(e)
        
        cv2.imshow("original Image",cvImage)
        cv2.waitKey(1)
        # Image segmentation 
        # Idea : https://www.thinkautonomous.ai/blog/lane-detection/
        h,w,c=cvImage.shape[0],cvImage.shape[1],cvImage.shape[2]
        # Create binary mask where the pixel of the Region of Interest are set to white
        #using a polygon covering the lane
        mask = np.zeros_like(cvImage)
        match_mask_color = (255,) * c    
        
        #polygon = np.array([[30,230],[90,160],[230,160],[w+50,w+50]]) 
        #polygon = np.array([[w+270,w],[h,250],[120,250],[-200,w]]) 
        polygon = np.array([[w,w],[h,165],[120,165],[30,h]]) 
        cv2.fillConvexPoly(mask, polygon, match_mask_color)
        
        masked_image = cv2.bitwise_and(cvImage, mask)
        cv2.imshow("blured Image",mask)
        cv2.waitKey(1)
        imgblur = cv2.GaussianBlur(masked_image, (5,5), 0)
        cv2.imshow("blured Image",imgblur)
        cv2.waitKey(1)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(imgblur, cv2.COLOR_BGR2HSV)

        # Define range of white color in HSV
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 30, 255])

        # Threshold the HSV image to get only white colors
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Define range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Combine masks
        mask = cv2.bitwise_or(mask_white, mask_yellow)

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
        t_lower = 50  # Lower Threshold 
        t_upper = 150  # Upper threshold 

        # Apply the canny filter to detect edges
        edges = cv2.Canny(gray,t_lower,t_upper)

        # Retrieve image's dimension
        length,width=edges.shape
        # Selecting the areas of interest (in this cas the robot requires the bottom part of the screen)
        # search_top = 7*length//8 #camera look directly beneath 
        # search_bot = 7*length//8 +80
        # # Setting the other regions to zero
        # edges[0:search_top, 0:width] = 0
        # edges[search_bot:length, 0:width ]= 0
        cv2.imshow("modified Image",edges)
        cv2.namedWindow('modified Image')
        cv2.setMouseCallback('modified Image', self.get_coordinate)
        
        cv2.waitKey(1)
        

        # Divide the image by half vertically
        # height, width = edges.shape
        # edges[0:height//2, 0:width]=0
        # Compute the moments
        M = cv2.moments(edges)
        # parse returned data from "moments"
        if M["m00"]>0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            
            # Update the coordinates   
            self.cX=cX
            self.cY=cY
            self.targetPixelCoordinate=1*width//2

            # draw the center of the shape on the image
            cv2.circle(edges, (cX, cY), 20, (255,255,255), -1)  #mark the center of the contour
            # draw a circle on the center of the image
            cv2.circle(edges, (self.targetPixelCoordinate, cY), 20, (150,150,255), -1)
            # Display the image with the circles on it 
            cv2.imshow("modified Image",edges)
            cv2.waitKey(1)

    def get_coordinate(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Clicked coordinate: ({}, {})".format(x, y))
    def publishMoments(self):
        """ This is the main function that will use all the methods 
            of the hereby class and publishes the moments
        """
        rate= rospy.Rate(100)
        
        while not rospy.is_shutdown():
            centroids= [self.cX,self.targetPixelCoordinate]
            self.pubCentroids.publish(Float32MultiArray(data=centroids))
        
        rate.sleep()    

if __name__ =='__main__':
    try:
        imageProcessing=ImageProcessingNode()
        imageProcessing.publishMoments()
    except rospy.ROSInterruptException:
        pass



