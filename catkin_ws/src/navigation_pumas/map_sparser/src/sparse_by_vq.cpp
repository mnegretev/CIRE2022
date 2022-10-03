#include "ros/ros.h"
#include "navig_msgs/Edges.h"
#include "navig_msgs/GeometryGraph.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "random_numbers/random_numbers.h"

random_numbers::RandomNumberGenerator rnd;

nav_msgs::OccupancyGrid get_inflated_map(nav_msgs::OccupancyGrid& map, float inflation)
{
    /*
     * WARNING!!! It is assumed that map borders (borders with at least 'inflation' thickness)
     * are occupied or unkwnon. Map must be big enough to fulfill this assumption.
     */
    if(inflation <= 0)
        return map;
    
    nav_msgs::OccupancyGrid newMap = map;
    int n = (int)(inflation / map.info.resolution);
    int lower_limit = n*map.info.width + n;
    int upper_limit = map.data.size() - n*map.info.width - n;
    for(int k = lower_limit; k < upper_limit; k++)
        if(map.data[k] != 0)
            for(int i=-n; i<=n; i++)
                for(int j=-n; j<=n; j++)
                    newMap.data[k + j*map.info.width + i] = map.data[k];

    return newMap;
}

std::vector<geometry_msgs::Point> get_free_points(nav_msgs::OccupancyGrid& map)
{
    std::vector<geometry_msgs::Point> points;
    for(size_t i=0; i < map.data.size(); i++)
    {
        if(map.data[i] != 0)
            continue;
        geometry_msgs::Point p;
        p.x = (i % map.info.width)*map.info.resolution + map.info.origin.position.x;
        p.y = (i / map.info.width)*map.info.resolution + map.info.origin.position.y;
        points.push_back(p);
    }
    return points;
}

std::vector<geometry_msgs::Point> disturb_centroids(std::vector<geometry_msgs::Point> centroids, float epsilon)
{
    std::vector<geometry_msgs::Point> new_centroids;
    for(size_t i=0; i < centroids.size(); i++)
    {
        float a = rnd.uniformReal(-M_PI, M_PI);
        geometry_msgs::Point p1,p2;
        p1.x = centroids[i].x + epsilon*cos(a);
        p1.y = centroids[i].y + epsilon*sin(a);
        p2.x = centroids[i].x - epsilon*cos(a);
        p2.y = centroids[i].y -epsilon*sin(a);
        new_centroids.push_back(p1);
        new_centroids.push_back(p2);
    }
    return new_centroids;
}

int get_nearest_centroid_idx(geometry_msgs::Point& p, std::vector<geometry_msgs::Point>& centroids)
{
    float min = INT_MAX;
    int nearest_idx = -1;
    for(size_t i=0; i < centroids.size(); i++)
    {
        float d = sqrt((p.x - centroids[i].x)*(p.x - centroids[i].x) + (p.y - centroids[i].y)*(p.y - centroids[i].y));
        if(d < min)
        {
            min = d;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

std::vector<geometry_msgs::Point> get_centroids(std::vector<geometry_msgs::Point>& points, int m, float epsilon, float tol)
{
    geometry_msgs::Point c;
    for(size_t i=0; i < points.size(); i++)
    {
        c.x += points[i].x;
        c.y += points[i].y;
    }
    c.x /= points.size();
    c.y /= points.size();
    std::cout << "MapSparser.->First centroid: " << c << std::endl;;
    std::vector<geometry_msgs::Point> centroids;
    centroids.push_back(c);
    while(centroids.size() < m && ros::ok())
    {
        centroids = disturb_centroids(centroids, epsilon);
        std::cout << "MapSparser.->Calculating " << centroids.size() << " centroids" << std::endl;
        float delta = tol + 1;
        while(delta > tol && ros::ok())
        {
            std::vector<geometry_msgs::Point> new_centroids;
            std::vector<int> counters;
            for(size_t i=0; i < centroids.size(); i++)
            {
                new_centroids.push_back(geometry_msgs::Point());
                counters.push_back(0);
            }
            for(size_t i=0; i < points.size(); i++)
            {
                int idx = get_nearest_centroid_idx(points[i], centroids);
                new_centroids[idx].x += points[i].x;
                new_centroids[idx].y += points[i].y;
                counters[idx]++;
            }
            delta = 0;
            for(size_t i=0; i < centroids.size(); i++)
            {
                new_centroids[i].x = counters[i] > 0 ? new_centroids[i].x/counters[i] : 0;
                new_centroids[i].y = counters[i] > 0 ? new_centroids[i].y/counters[i] : 0;
                delta += fabs(new_centroids[i].x - centroids[i].x) + fabs(new_centroids[i].y - centroids[i].y);
            }
            delta /= centroids.size();
            std::cout << "MapSparser.->Current delta: " << delta << std::endl;
            centroids = new_centroids;
        }
    }
    return centroids;
}

bool are_visible(geometry_msgs::Point a, geometry_msgs::Point b, nav_msgs::OccupancyGrid& map)
{
    float m  = sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
    float dx = (b.x - a.x)/m*map.info.resolution/2;
    float dy = (b.y - a.y)/m*map.info.resolution/2;
    int   n  = (int)(m/map.info.resolution*2);
    for(int i=0; i < n; i++)
    {
        float x = a.x + dx*i;
        float y = a.y + dy*i;
        int ix = (int)((x - map.info.origin.position.x)/map.info.resolution);
        int iy = (int)((y - map.info.origin.position.y)/map.info.resolution);
        int ic = iy*map.info.width + ix;
        if(map.data[ic] != 0)
            return false;
    }
    return true;
}

navig_msgs::GeometryGraph build_graph(std::vector<geometry_msgs::Point>& centroids, nav_msgs::OccupancyGrid& map)
{
    navig_msgs::GeometryGraph graph;
    

    
    graph.nodes = centroids;
    graph.edges.resize(graph.nodes.size());
    for(size_t i=0; i<centroids.size(); i++)
        for(size_t j=0; j<centroids.size(); j++)
        {
            if(i==j) continue;
            if(are_visible(centroids[i], centroids[j], map))
            {
                graph.edges[i].node_ids.push_back(j);
                graph.edges[i].weights.push_back(sqrt(pow(centroids[i].x - centroids[j].x,2)+pow(centroids[i].y - centroids[j].y,2)));
            }
        }
    graph.header.frame_id = "map";
    graph.header.stamp    = ros::Time::now();
    return graph;
}

visualization_msgs::Marker get_nodes_marker(std::vector<geometry_msgs::Point>& centroids)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sparse_map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for(size_t i=0; i < centroids.size(); i++)
    {
        try{
            geometry_msgs::Point p;
            p.x = centroids[i].x;
            p.y = centroids[i].y;
            p.z = 0.13;
            marker.points.push_back(p);
        }catch(...){}
    }
    return marker;
}

visualization_msgs::Marker get_edges_marker(navig_msgs::GeometryGraph& graph)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sparse_map";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for(size_t i=0; i < graph.edges.size(); i++)
        for(size_t j=0; j < graph.edges[i].node_ids.size(); j++)
        {
            marker.points.push_back(graph.nodes[i]);
            marker.points.push_back(graph.nodes[graph.edges[i].node_ids[j]]);
        }
    return marker;
}

int main(int argc, char** argv)
{
    std::cout << "MapSparser.->INITIALIZING MAP SPARSER..." << std::endl;
    ros::init(argc, argv, "map_sparser");
    ros::NodeHandle n;

    int   num_centroids = 256;
    float epsilon       = 0.1;
    float tolerance     = 0.05;
    float inflation_radius = 0.25;
    if(ros::param::has("~centroids"))
        ros::param::get("~centroids", num_centroids);
    if(ros::param::has("~epsilon"))
        ros::param::get("~epsilon", epsilon);
    if(ros::param::has("~tolerance"))
        ros::param::get("~tolerance", tolerance);
    if(ros::param::has("~inflation_radius"))
        ros::param::get("~inflation_radius", inflation_radius);
    
    std::cout << "MapSparser.->Waiting for static map topic..." << std::endl;
    ros::service::waitForService("/static_map", ros::Duration(1000.0));
    ros::ServiceClient cltGetStaticMap = n.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap srvStaticMap;
    cltGetStaticMap.call(srvStaticMap);
    std::cout << "MapSparser.->Static map received." << std::endl;
    std::cout << "MapSparser.->Inflating map by " << inflation_radius << " ..." << std::endl;
    nav_msgs::OccupancyGrid map = get_inflated_map(srvStaticMap.response.map, inflation_radius);
    std::vector<geometry_msgs::Point> points = get_free_points(map);
    std::cout << "MapSparser.->Inflated map with " << points.size() << " free cells" << std::endl;
    std::cout << "MapSparser.->Calculating centroids by vector quantization..." << std::endl;
    std::vector<geometry_msgs::Point> centroids = get_centroids(points, num_centroids, epsilon, tolerance);
    std::cout << "MapSparser.->Building graph from centroids and map..." << std::endl;
    navig_msgs::GeometryGraph graph = build_graph(centroids, map);
    

    ros::Rate loop(1);
    ros::Publisher pub_marker = n.advertise<visualization_msgs::Marker>("/sparse_map", 10);
    
    while(ros::ok())
    {
        pub_marker.publish(get_nodes_marker(centroids));
        pub_marker.publish(get_edges_marker(graph));
        loop.sleep();
    }
    return 0;
}
