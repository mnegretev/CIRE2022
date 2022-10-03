#!/usr/bin/env python
import numpy
import rospy
from geometry_msgs.msg import Point 
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    for i in range(height):
        for j in range(width):
            if static_map[i,j] > 0:
                for k1 in range(-inflation_cells, inflation_cells+1):
                    for k2 in range(-inflation_cells, inflation_cells+1):
                        inflated[i+k1, j+k2] = 100
    return inflated


def disturb_centroids(centroids, epsilon):
    new_centroids = []
    for c in centroids:
        v = numpy.random.rand(2)
        v = epsilon * v / numpy.linalg.norm(v)
        new_centroids.append(c + v)
        new_centroids.append(c - v)
    return new_centroids

def get_centroids(points, m, epsilon, tol):
    centroids = [numpy.mean(points, axis=0)]
    print("MapSparser.->First centroid: " + str(centroids))
    while len(centroids) < m and not rospy.is_shutdown():
        centroids = disturb_centroids(centroids, epsilon)
        print("MapSparser.->Calculating " + str(len(centroids)) + " centroids")
        delta = tol*len(centroids) + 1
        while delta > tol*len(centroids) and not rospy.is_shutdown():
            clusters = [[] for c in centroids]
            for p in points:
                idx = numpy.argmin([numpy.linalg.norm(p-c) for c in centroids])
                clusters[idx].append(p)
            new_centroids = [numpy.mean(c, axis=0) for c in clusters]
            delta = numpy.sum([numpy.linalg.norm(new_centroids[i] - centroids[i]) for i in range(len(centroids))])
            centroids = new_centroids
            print("MapSparser.->Current delta: " + str(delta))
    return centroids

def get_map_points(static_map):
    points = []
    for i in range(len(static_map.data)):
        if static_map.data[i] != 0:
            continue
        x = (i %static_map.info.width)*static_map.info.resolution + static_map.info.origin.position.x
        y = (i//static_map.info.width)*static_map.info.resolution + static_map.info.origin.position.y
        points.append([x,y])
    return numpy.asarray(points)

def get_marker(centroids):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "map_centroids"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    for c in centroids:
        try:
            p = Point()
            p.x = c[0]
            p.y = c[1]
            marker.points.append(p)
        except:
            pass
    return marker

def main():
    print("INITIALIZING MAP SPARSER...")
    rospy.init_node("map_sparser")
    centroids = 4
    epsilon   = 0.1
    tolerance = 0.1
    inflation_radius = 0.3
    if rospy.has_param("~centroids"):
        centroids = rospy.get_param("~centroids")
    if rospy.has_param("~epsilon"):
        epsilon   = rospy.get_param("~epsilon")
    if rospy.has_param("~tolerance"):
        tolerance = rospy.get_param("~tolerance")
    if rospy.has_param("~inflation_radius"):
        inflation_radius = rospy.get_param("~inflation_radius")
        
    print("MapSparser.->Waiting for static map topic...")
    static_map = rospy.wait_for_message("/map", OccupancyGrid)
    print("MapSparser.->Static map received.")
    grid = numpy.asarray(static_map.data, dtype='int')
    grid = numpy.reshape(grid, (static_map.info.height, static_map.info.width))
    inflation_cells = int(inflation_radius/static_map.info.resolution)
    print("MapSparser.->Inflating map with " + str(inflation_cells) + " cells.")
    inflated_grid = get_inflated_map(grid, inflation_cells)
    inflated_map  = OccupancyGrid()
    inflated_map.info = static_map.info
    inflated_map.data = numpy.ravel(numpy.reshape(inflated_grid, (len(static_map.data), 1)))
    
    points = get_map_points(inflated_map)
    print("MapSparser.->Inflated map with " + str(points.shape) + " occupied cells")
    centroids = get_centroids(points, centroids, epsilon, tolerance)

    loop = rospy.Rate(1)
    pub_centroids = rospy.Publisher("/sparse_map", Marker, queue_size=1)
    while not rospy.is_shutdown():
        pub_centroids.publish(get_marker(centroids))
        loop.sleep()

if __name__ == '__main__':
    main()
