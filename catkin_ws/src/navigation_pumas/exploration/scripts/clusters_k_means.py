#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from navig_msgs.srv import GetBoundaryPoints,GetBoundaryPointsResponse
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import Point
import math



class Server:

    def __init__(self):
        
        self.boundary_points=[]
        
        


    def GetBoundaryPoints(self,req):
        
            #----------------Boundary Points Client--------------------------------
            #print("Establishing the connection with Boundary Points Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points')#We are waiting to the connection with the server
            try:
                
                client_boundary_points=rospy.ServiceProxy('/navigation/mapping/get_boundary_points',GetBoundaryPoints)#We create the handler of the Service
                data_bp=client_boundary_points(map=req.map)#We call the service
                self.boundary_points=data_bp.points
        
            except rospy.ServiceException as e:
                print("The request for the boundary points server failed: %s"%e)

            
            #print("Already we get the data related with the GetBoundaryPoints service\n")
            #print("The number of Boundary Points founded are "+str(len(self.boundary_points))+"\n")


   
    def handle_GetCentroids(self,req):

        
        self.GetBoundaryPoints(req)
        points=[]
        std_dev=[]
        for i in range(len(self.boundary_points)):
            points.append([self.boundary_points[i].x,self.boundary_points[i].y])

        np.asarray(points)


        for i in range(1, 9):#We will adjust the model to different values of k, in order to select the best number of clusters
            kmeans = KMeans(n_clusters = i, init = "k-means++", max_iter = 300, n_init = 10, random_state = 0)
            kmeans.fit(points)
            std_dev.append(math.sqrt(kmeans.inertia_/len(points)))#We are calculating the standard deviation of each model depending of the number of clusters k
        
        
        for i in range(len(std_dev)):
            #We will select the number of clusters based on the standard deviation, in this case, we want clusters close to 1 meter
            if std_dev[i]>1:
                k=i+2
            
        #We will adjust the model to the number of clusters selected
        kmeans = KMeans(n_clusters = k, init="k-means++", max_iter = 300, n_init = 10, random_state = 0)
        kmeans.fit(points)
        centroids_x=kmeans.cluster_centers_[:,0]
        centroids_y=kmeans.cluster_centers_[:,1]
        centroids=[]
        #We will define the Points that will have the coordinates of the centroids found
        for i in range(len(centroids_x)):
            p=Point()
            p.x=centroids_x[i]
            p.y=centroids_y[i]
            p.z=0
            centroids.append(p)
            
        #print("We already get the centroids\n")
        return GetBoundaryPointsResponse(points=centroids)
        

    def GetCentroids(self):
            
        rospy.Service('/navigation/mapping/get_boundary_points_clustered', GetBoundaryPoints, self.handle_GetCentroids)
        #print("The Clusters K Means Server is ready for the request")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('clusters_k_means')
    server=Server()
    server.GetCentroids()
    rospy.spin()
    
