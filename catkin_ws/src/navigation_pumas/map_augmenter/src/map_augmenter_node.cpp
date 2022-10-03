#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "std_srvs/Trigger.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

float minX =  0.3;
float maxX =  0.7;
float minY = -0.3;
float maxY =  0.3;
float minZ =  0.1;
float maxZ =  1.5;
float inflation_radius = 0.25;
float cost_radius = 0.5;
int   cloud_downsampling  = 9;;
int   cloud_downsampling2 = 9;;
int   lidar_downsampling  = 2;
bool  are_there_obstacles = false;

bool use_lidar  = true;
bool use_sonars = false;
bool use_cloud  = false;
bool use_cloud2 = false;

tf::TransformListener* listener;
std::string base_link_name      = "/base_footprint";
std::string point_cloud_topic   = "/point_cloud";
std::string point_cloud_frame   = "/point_cloud_frame";
std::string point_cloud_topic2  = "/point_cloud2";
std::string point_cloud_frame2  = "/point_cloud_frame2";
std::string laser_scan_topic    = "/scan";
std::string laser_scan_frame    = "laser";
std::string sonars_topic_prefix = "/sonar";

nav_msgs::OccupancyGrid static_map;             //Static map plus prohibition layer with inflation. Calculated once on node start.
nav_msgs::OccupancyGrid static_cost_map;        //Cost map calculated using inflated static map and prohibition layer. Calculated once on node start.
nav_msgs::OccupancyGrid obstacles_map;          //Only obstacles map calculated with all enabled sensors, without inflation. Calculated on service request.
nav_msgs::OccupancyGrid obstacles_inflated_map; //Only obstacles with inflation.
nav_msgs::OccupancyGrid augmented_map;          //Static map plus prohibition layer plus obstacles map plus inflation. Calculated on service request.

Eigen::Affine3d get_robot_position()
{
    tf::StampedTransform tf;
    listener->lookupTransform("map", base_link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void get_robot_position(float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    listener->lookupTransform("map", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

Eigen::Affine3d get_camera_position()
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, point_cloud_frame, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

Eigen::Affine3d get_camera_position2()
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, point_cloud_frame2, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

Eigen::Affine3d get_lidar_position()
{
    tf::StampedTransform tf;
    listener->lookupTransform(base_link_name, laser_scan_frame, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

nav_msgs::OccupancyGrid merge_maps(nav_msgs::OccupancyGrid& a, nav_msgs::OccupancyGrid& b)
{
    if(a.info.width != b.info.width || a.info.height != b.info.height)
    {
        std::cout << "MapAugmenter.->WARNING!!! Cannot merge maps of different sizes!!!" <<std::endl;
        return a;
    }
    nav_msgs::OccupancyGrid c = a;
    for(size_t i=0; i< c.data.size(); i++)
        c.data[i] = (char)std::max((unsigned char)a.data[i], (unsigned char)b.data[i]);
    return c;
}

nav_msgs::OccupancyGrid inflate_map(nav_msgs::OccupancyGrid& map, float inflation)
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
        if(map.data[k] > 0)
            for(int i=-n; i<=n; i++)
                for(int j=-n; j<=n; j++)
                    newMap.data[k + j*map.info.width + i] = map.data[k];

    return newMap;
}

nav_msgs::OccupancyGrid get_cost_map(nav_msgs::OccupancyGrid& map, float cost_radius)
{
    if(cost_radius < 0)
        return map;

    nav_msgs::OccupancyGrid cost_map = map;
    int steps = (int)(cost_radius / map.info.resolution);
    int boxSize = (steps*2 + 1) * (steps*2 + 1);
    int* cell_costs = new int[boxSize];
    int* neighbors  = new int[boxSize];
    int startIdx = steps*map.info.width + steps;
    int endIdx = map.data.size() - steps*map.info.width - steps;
    
    int counter = 0;
    for(int i=-steps; i<=steps; i++)
        for(int j=-steps; j<=steps; j++)
        {
            neighbors [counter] = i*map.info.width + j;
            cell_costs[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1)*2;
            counter++;
        }

    for(size_t i=startIdx; i < endIdx; i++)
        if(map.data[i] > 0)
            for(int j = 0; j < boxSize; j++)
                if(cost_map.data[i+neighbors[j]] < cell_costs[j])
                    cost_map.data[i+neighbors[j]] = cell_costs[j];

    delete[] cell_costs;
    delete[] neighbors;
    return cost_map;
}

bool obstacles_map_with_cloud()
{
    std::cout << "MapAugmenter.->Trying to get point cloud from topic: " << point_cloud_topic << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic, ros::Duration(5.0));
    if(ptr == NULL)
    {
        std::cout << "MapAugmenter.->Cannot get point cloud!!!" << std::endl;
        return false;
    }
    unsigned char* p = (unsigned char*)(&ptr->data[0]);
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d cam_to_robot = get_camera_position();
    Eigen::Affine3d robot_to_map = get_robot_position();

    for(size_t i=0; i < ptr->width*ptr->height; i+=cloud_downsampling)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > minX && v.x() < maxX && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ)
        {
            v = robot_to_map * v;
            cell_x = (int)((v.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
            cell_y = (int)((v.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
            cell   = cell_y * obstacles_map.info.width + cell_x;
            obstacles_map.data[cell] = 100;
            are_there_obstacles = true;
        }
        p += cloud_downsampling*ptr->point_step;
    }
    return true;
}

bool obstacles_map_with_cloud2()
{
    std::cout << "MapAugmenter.->Trying to get point cloud2 from topic: " << point_cloud_topic2 << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr=ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic2, ros::Duration(1.0));
    if(ptr == NULL)
    {
        std::cout << "MapAugmenter.->Cannot get point cloud2!!!" << std::endl;
        return false;
    }
    unsigned char* p = (unsigned char*)(&ptr->data[0]);
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d cam_to_robot = get_camera_position2();
    Eigen::Affine3d robot_to_map = get_robot_position();

    for(size_t i=0; i < ptr->width*ptr->height; i+=cloud_downsampling2)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > minX && v.x() < maxX && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ)
        {
            v = robot_to_map * v;
            cell_x = (int)((v.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
            cell_y = (int)((v.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
            cell   = cell_y * obstacles_map.info.width + cell_x;
            obstacles_map.data[cell] = 100;
            are_there_obstacles = true;
        }
        p += cloud_downsampling2*ptr->point_step;
    }
    return true;
}

bool obstacles_map_with_lidar()
{
    std::cout << "MapAugmenter.->Trying to get laser scan from topic: " << laser_scan_topic << std::endl;
    boost::shared_ptr<sensor_msgs::LaserScan const> ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_scan_topic, ros::Duration(1.0));
    if(ptr == NULL)
    {
        std::cout << "MapAugmenter.->Cannot get laser scan!!!" << std::endl;
        return false;
    }
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;

    Eigen::Affine3d lidar_to_robot = get_lidar_position();
    Eigen::Affine3d robot_to_map   = get_robot_position();

    for(size_t i=0; i < ptr->ranges.size(); i+=lidar_downsampling)
    {
        float angle = ptr->angle_min + i*ptr->angle_increment;
        Eigen::Vector3d v(ptr->ranges[i]*cos(angle), ptr->ranges[i]*sin(angle), 0);
        v = lidar_to_robot * v;
        if(v.x() > minX && v.x() < maxX && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ) 
        {
            v = robot_to_map * v;
            cell_x = (int)((v.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
            cell_y = (int)((v.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
            cell   = cell_y * obstacles_map.info.width + cell_x;
            obstacles_map.data[cell] = 100;
            are_there_obstacles = true;
        }
    }
    return true;
}

bool obstacles_map_with_sonars()
{
    std::cout << "MapAugmenter.->Trying to get sonars from topic prefix: " << sonars_topic_prefix << std::endl;
    boost::shared_ptr<sensor_msgs::Range const> ptr0 = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"0", ros::Duration(1.0));
    boost::shared_ptr<sensor_msgs::Range const> ptr1 = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"1", ros::Duration(1.0));
    boost::shared_ptr<sensor_msgs::Range const> ptr2 = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"2", ros::Duration(1.0));
    if(ptr0 == NULL || ptr1 == NULL || ptr2 == NULL)
    {
        std::cout << "MapAugmenter.->Cannot get sonar readings!!!" << std::endl;
        return false;
    }
    int cell_x = 0;
    int cell_y = 0;
    int cell   = 0;
    Eigen::Vector3d v0(ptr0->range*cos( 0.3), ptr0->range*sin( 0.3), 0);
    Eigen::Vector3d v1(ptr1->range         , 0                     , 0);
    Eigen::Vector3d v2(ptr2->range*cos(-0.3), ptr2->range*sin(-0.3), 0);
    Eigen::Affine3d robot_to_map   = get_robot_position();
    if(v0.x() > minX && v0.x() < maxX && v0.y() > minY && v0.y() < maxY) 
    {
        v0 = robot_to_map * v0;
        cell_x = (int)((v0.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
        cell_y = (int)((v0.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
        cell   = cell_y * obstacles_map.info.width + cell_x;
        are_there_obstacles = true;
        obstacles_map.data[cell] = 100;
    }
    if(v1.x() > minX && v1.x() < maxX && v1.y() > minY && v1.y() < maxY) 
    {
        v1 = robot_to_map * v1;
        cell_x = (int)((v1.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
        cell_y = (int)((v1.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
        cell   = cell_y * obstacles_map.info.width + cell_x;
        obstacles_map.data[cell] = 100;
        are_there_obstacles = true;
    }
    if(v2.x() > minX && v2.x() < maxX && v2.y() > minY && v2.y() < maxY) 
    {
        v2 = robot_to_map * v2;
        cell_x = (int)((v2.x() - obstacles_map.info.origin.position.x)/obstacles_map.info.resolution);
        cell_y = (int)((v2.y() - obstacles_map.info.origin.position.y)/obstacles_map.info.resolution);
        cell   = cell_y * obstacles_map.info.width + cell_x;
        obstacles_map.data[cell] = 100;
        are_there_obstacles = true;
    }
    return true;
}

bool callback_static_map(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& resp)
{
    resp.map = static_map;
    return true;
}

bool callback_augmented_map(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& resp)
{
    std::cout << "MapAugmenter.->Augmenting map using: "<<(use_lidar?"lidar ":"")<<(use_sonars?"sonars ":"");
    std::cout << (use_cloud? "point_cloud ":"") << (use_cloud2? "point_cloud2":"") <<std::endl;
    if(use_lidar  && !obstacles_map_with_lidar())
        return false;
    if(use_sonars && !obstacles_map_with_sonars())
        return false;
    if(use_cloud  &&!obstacles_map_with_cloud ())
        return false;
    if(use_cloud2 &&!obstacles_map_with_cloud2())
        return false;
    
    obstacles_inflated_map = inflate_map(obstacles_map, inflation_radius);
    augmented_map = merge_maps(static_map, obstacles_inflated_map);
    resp.map = augmented_map;
    return true;
}

bool callback_static_cost_map(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& resp)
{
    resp.map = static_cost_map;
    return true;
}

bool callback_augmented_cost_map(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& resp)
{
    nav_msgs::OccupancyGrid obs_cost_map = get_cost_map(obstacles_inflated_map, cost_radius);
    resp.map = merge_maps(static_cost_map, obs_cost_map);
    return true;
}

bool callback_are_there_obstacles(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = are_there_obstacles;
    return true;
}

bool callback_is_inside_obstacles(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    float robot_x, robot_y, robot_a;
    get_robot_position(robot_x, robot_y, robot_a);
    int cell_x = (int)((robot_x - augmented_map.info.origin.position.x)/augmented_map.info.resolution);
    int cell_y = (int)((robot_y - augmented_map.info.origin.position.y)/augmented_map.info.resolution);
    int cell   = cell_y * obstacles_map.info.width + cell_x;
    resp.success = augmented_map.data[cell] > 0;
    return true;
}

bool decay_map_and_check_if_obstacles(nav_msgs::OccupancyGrid& map, int decay_factor)
{
    bool obstacles = false;
    for(size_t i=0; i < map.data.size(); i++)
    {
        map.data[i] -= decay_factor;
        if(map.data[i] < 0)
            map.data[i] = 0;
        obstacles |= map.data[i] > 0;
    }
    return obstacles;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING MAP AUGMENTER BY MARCO NEGRETE..." << std::endl;
    ros::init(argc, argv, "map_augmenter");
    ros::NodeHandle n;
    ros::Publisher pubAugmentedMap = n.advertise<nav_msgs::OccupancyGrid>("/augmented_map", 10);
    listener = new tf::TransformListener();
    ros::Rate loop(10);

    int decay_factor = 10;
    if(ros::param::has("~use_lidar"))
        ros::param::get("~use_lidar", use_lidar);
    if(ros::param::has("~use_sonars"))
        ros::param::get("~use_sonars", use_sonars);
    if(ros::param::has("~use_point_cloud"))
        ros::param::get("~use_point_cloud", use_cloud);
    if(ros::param::has("~use_point_cloud2"))
        ros::param::get("~use_point_cloud2", use_cloud2);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", minX);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", maxX);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", minY);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", maxY);
    if(ros::param::has("~min_z"))
        ros::param::get("~min_z", minZ);
    if(ros::param::has("~max_z"))
        ros::param::get("~max_z", maxZ);
    if(ros::param::has("~point_cloud_topic"))
        ros::param::get("~point_cloud_topic", point_cloud_topic);
    if(ros::param::has("~point_cloud_topic2"))
        ros::param::get("~point_cloud_topic2", point_cloud_topic2);
    if(ros::param::has("~laser_scan_topic"))
        ros::param::get("~laser_scan_topic",  laser_scan_topic);
    if(ros::param::has("~sonars_topic_prefix"))
        ros::param::get("~sonars_topic_prefix", sonars_topic_prefix);
    if(ros::param::has("~decay_factor"))
        ros::param::get("~decay_factor", decay_factor);
    if(ros::param::has("~inflation_radius"))
    	ros::param::get("~inflation_radius", inflation_radius);
    if(ros::param::has("~cost_radius"))
    	ros::param::get("~cost_radius", cost_radius);
    if(ros::param::has("~cloud_downsampling"))
        ros::param::get("~cloud_downsampling", cloud_downsampling);
    if(ros::param::has("~cloud_downsampling2"))
        ros::param::get("~cloud_downsampling2", cloud_downsampling2);
    if(ros::param::has("~lidar_downsampling"))
        ros::param::get("~lidar_downsampling", lidar_downsampling);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);

    std::cout << "MapAugmenter.->Parameters: min_x =" << minX << "  max_x =" << maxX << "  min_y =" << minY << "  max_y =" << maxY;
    std::cout << "  min_z =" << minZ << "  max_z =" << maxZ << std::endl;
    std::cout << "MapAugmenter.->Parameters: decay_factor="<<decay_factor<<"  inflation="<<inflation_radius<<"  cost_radius="<<cost_radius << std::endl;
    std::cout << "MapAugmenter.->Parameters: cloud_downsampling="<< cloud_downsampling << "  cloud_downsampling2=" << cloud_downsampling2;
    std::cout << "  lidar_downsampling=" << lidar_downsampling << "  base link name: " << base_link_name << std::endl;

    std::cout << "MapAugmenter.->Waiting for service to get static map..." << std::endl;
    ros::service::waitForService("/static_map"     , ros::Duration(10000.0));
    ros::service::waitForService("/prohibition_map", ros::Duration(10000.0));
    ros::ServiceClient cltGetStaticMap = n.serviceClient<nav_msgs::GetMap>("/static_map");
    ros::ServiceClient cltGetProhibitionMap = n.serviceClient<nav_msgs::GetMap>("/prohibition_map");
    nav_msgs::GetMap srvMap;
    nav_msgs::GetMap srvProhibitionMap;
    if(!cltGetStaticMap.call(srvMap))
    {
        std::cout << "MapAugmenter.->Cannot get static map!!!!" << std::endl;
        return -1;
    }
    if(!cltGetProhibitionMap.call(srvProhibitionMap))
    {
        std::cout << "MapAugmenter.->Cannot get prohibition map!!!!" << std::endl;
        return -1;
    }

    std::cout << "MapAugmenter.->Generating static map with prohibition layer and static cost map..." << std::endl;
    static_map = merge_maps(srvMap.response.map, srvProhibitionMap.response.map);
    static_map = inflate_map(static_map, inflation_radius);
    static_cost_map = get_cost_map(static_map, cost_radius);
    obstacles_map = static_map;
    for(size_t i=0; i < obstacles_map.data.size(); i++) obstacles_map.data[i] = 0;
    std::cout << "MapAugmenter.->Statics maps are ready." << std::endl;
    
    std::cout << "MapAugmenter.->Trying to get first messages from active sensor topics: " << (use_cloud ? point_cloud_topic : "") << "  ";
    std::cout << (use_cloud2?point_cloud_topic2 : "")<<" "<<(use_lidar?laser_scan_topic : "")<<" "<<(use_sonars ? sonars_topic_prefix:"") << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr_c1  ; 
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr_c2  ;
    boost::shared_ptr<sensor_msgs::LaserScan const>   ptr_scan;
    boost::shared_ptr<sensor_msgs::Range const>       ptr0    ;
    boost::shared_ptr<sensor_msgs::Range const>       ptr1    ;
    boost::shared_ptr<sensor_msgs::Range const>       ptr2    ;
    if(use_cloud)  ptr_c1   = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic , ros::Duration(10000.0));
    if(use_cloud2) ptr_c2   = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic2, ros::Duration(10000.0));
    if(use_lidar)  ptr_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(laser_scan_topic, ros::Duration(10000.0));
    if(use_sonars) ptr0     = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"0", ros::Duration(10000.0));
    if(use_sonars) ptr1     = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"1", ros::Duration(10000.0));
    if(use_sonars) ptr2     = ros::topic::waitForMessage<sensor_msgs::Range>(sonars_topic_prefix+"2", ros::Duration(10000.0));
    std::cout << "MapAugmenter.->First messages received..." << std::endl;
    if(use_cloud  && ptr_c1   != NULL) point_cloud_frame  = ptr_c1->header.frame_id;
    if(use_cloud2 && ptr_c2   != NULL) point_cloud_frame2 = ptr_c2->header.frame_id;
    if(use_lidar  && ptr_scan != NULL) laser_scan_frame   = ptr_scan->header.frame_id;

    std::cout << "MapAugmenter.->Waiting for localization transform" << std::endl;
    listener->waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(1000.0));
    std::cout << "MapAugmenter.->Localization transform is now available"<< std::endl;
    std::cout << "MapAugmenter.->Waiting for sensor transforms" << std::endl;
    if (use_cloud ) listener->waitForTransform(base_link_name, point_cloud_frame , ros::Time(0), ros::Duration(1000.0));
    if (use_cloud2) listener->waitForTransform(base_link_name, point_cloud_frame2, ros::Time(0), ros::Duration(1000.0));
    if (use_lidar)  listener->waitForTransform(base_link_name, laser_scan_frame,   ros::Time(0), ros::Duration(1000.0));
    std::cout << "MapAugmenter.->Sensor transforms are now available"<< std::endl;
    
    ros::ServiceServer srvStaticMap         = n.advertiseService("/map_augmenter/get_static_map"        , callback_static_map);
    ros::ServiceServer srvStaticCostMap     = n.advertiseService("/map_augmenter/get_static_cost_map"   , callback_static_cost_map);
    ros::ServiceServer srvAugmentedMap      = n.advertiseService("/map_augmenter/get_augmented_map"     , callback_augmented_map);
    ros::ServiceServer srvAugmentedCostMap  = n.advertiseService("/map_augmenter/get_augmented_cost_map", callback_augmented_cost_map);
    ros::ServiceServer srvAreThereObstacles = n.advertiseService("/map_augmenter/are_there_obstacles"   , callback_are_there_obstacles);
    ros::ServiceServer srvIsInsideObstacles = n.advertiseService("/map_augmenter/is_inside_obstacles"   , callback_is_inside_obstacles);
        
    int counter = 0;
    while(ros::ok())
    {
        if(++counter > 10)
        {
            counter = 0;
            are_there_obstacles = decay_map_and_check_if_obstacles(obstacles_map, decay_factor);
            obstacles_inflated_map = inflate_map(obstacles_map, inflation_radius);
            augmented_map = merge_maps(static_map, obstacles_inflated_map);
            pubAugmentedMap.publish(augmented_map);
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
