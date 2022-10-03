#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from navig_msgs.srv import GetBoundaryPoints, GetBoundaryPointsResponse
import numpy as np
import cv2
from geometry_msgs.msg import Point

class Server():

    
        

    def handle_GetBoundaryPoints(self,req):
        
        inflated_map=np.array(req.map.data).reshape((req.map.info.height, req.map.info.width))#We recive the map and we reshape it into a heightxwidth array
        inflated_map = np.uint8(inflated_map)
        #We apply the Sobel filter to get the borders of the image in the x and y direction
        x=cv2.Sobel(inflated_map,cv2.CV_16S,1,0)
        y=cv2.Sobel(inflated_map,cv2.CV_16S,0,1)
        absX = cv2.convertScaleAbs(x)
        absY = cv2.convertScaleAbs(y)
        #The addweighted function helps in this transition of the image to another. 
        #In order to blend this image, we can add weights are define the transparency and translucency of the images.
        borders= cv2.addWeighted(absX,0.2,absY,0.2,0)  
        #We have simple thresholding where we manually supply parameters to segment the image
        ret,borders=cv2.threshold(borders,90,255,cv2.THRESH_BINARY)
        kernel = np.ones((2,2),np.uint8)
        #It is just opposite of erosion. Here, a pixel element is 1 if atleast one pixel under the kernel is 1.
        #So it increases the white region in the image or size of foreground object increases.
        #Normally, in cases like noise removal, erosion is followed by dilation. 
        #Because, erosion removes white noises, but it also shrinks our object.
        #So we dilate it.
        borders=cv2.dilate(borders,kernel)
        #The kernel slides through the image (as in 2D convolution). 
        #A pixel in the original image (either 1 or 0) will be considered 1 only if all the pixels under the kernel is 1, otherwise it is eroded (made to zero).
        borders=cv2.erode(borders,kernel)
        borders=cv2.erode(borders,kernel)
        #We check the coordinates of the borders and transformed into meters 
        borders_y=(np.where(borders==255)[0])*req.map.info.resolution+req.map.info.origin.position.y
        borders_x=(np.where(borders==255)[1])*req.map.info.resolution+req.map.info.origin.position.x
        boundary_points=[]
        #We construct all the points that save the borders 
        for i in range(len(borders_x)):
            p=Point()
            p.x=borders_x[i]
            p.y=borders_y[i]
            boundary_points.append(p)
            
            
        
        
        #print("We already get the boundary points\n")
        return GetBoundaryPointsResponse(points=boundary_points)#We return the Points array

        

    def GetBoundaryPoints(self):

        rospy.Service('/navigation/mapping/get_boundary_points', GetBoundaryPoints, self.handle_GetBoundaryPoints)
        #print("The Boundary Points Server is ready for the request")
        
        
        
if __name__ == "__main__":
    rospy.init_node('boundary_points')
    server=Server()
    server.GetBoundaryPoints()
    rospy.spin()
    
    
