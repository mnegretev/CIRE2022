#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
import rospy
from nav_msgs.msg import OccupancyGrid
from navig_msgs.srv import GetInflatedMap, GetInflatedMapResponse
import numpy as np



class Server:

    def __init__(self):
        
       self.inflated_map=[]


    def handle_GetInflatedMap(self,req):
        
        map=np.array(req.map.data).reshape((req.map.info.height, req.map.info.width))#We recive the map and we reshape it into a heightxwidth array
        inflated_cells=int(req.inflated_cells/req.map.info.resolution)
        #print ("Getting the inflated map with " +str(inflated_cells) + " inflated cells\n")
        pub_inflated=rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)#We define the publisher to publish the inflated map and visualaze it in rviz
        c, l=np.shape(map)
        self.inflated_map=np.copy(map)
        
        #Algorithm to inflate the map
        for i in range(c):
            for j in range(l): 
                if map[i,j]==100:
                    for k1 in range(i-inflated_cells,i+inflated_cells+1):
                        for k2 in range(j-inflated_cells,j+inflated_cells+1):
                            self.inflated_map[k1,k2]=map[i,j]
        
        
        
        #We reshape the inflated map to a 1-D array and define the attribute data of the OccupancyGrid with it
        req.map.data= np.ravel(np.reshape(self.inflated_map, (len(req.map.data), 1)))
        pub_inflated.publish(req.map)
        
        
        return GetInflatedMapResponse(inflated_map=req.map)




    def GetInflatedMap(self):

        rospy.Service('/navigation/mapping/get_inflated_map', GetInflatedMap, self.handle_GetInflatedMap)
        #print("The Inflated Map Server is ready for the request")
      
        
    
if __name__ == "__main__":
    rospy.init_node('inflated_map')
    server=Server()
    server.GetInflatedMap()
    rospy.spin()
    
    

    
      
        
    
