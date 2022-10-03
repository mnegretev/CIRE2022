#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#define RATE 30

bool  debug = false;
bool  enable = false;
bool use_lidar  = true;
bool use_sonars = false;
bool use_cloud  = false;
bool use_cloud2 = false;
float current_speed_linear  = 0;
float current_speed_angular = 0;
float minX =  0.3;
float maxX =  0.7;
float minY = -0.3;
float maxY =  0.3;
float minZ =  0.1;
float maxZ =  1.5;
int   cloud_downsampling      = 9;
int   cloud_points_threshold  = 100;
int   cloud_downsampling2     = 9;
int   cloud_points_threshold2 = 100;
int   lidar_downsampling      = 2;
int   lidar_points_threshold  = 15;
std::string base_link_name      = "base_footprint";
std::string point_cloud_topic   = "/point_cloud";
std::string point_cloud_frame   = "/point_cloud_frame";
std::string point_cloud_topic2  = "/point_cloud2";
std::string point_cloud_frame2  = "/point_cloud_frame2";
std::string laser_scan_topic    = "/scan";
std::string laser_scan_frame    = "laser";
std::string sonars_topic_prefix = "/sonar";
int no_data_cloud_counter  = 0;
int no_data_cloud_counter2 = 0;
int no_data_lidar_counter  = 0;
int no_data_sonars_counter = 0;

ros::NodeHandle* nh;
ros::Subscriber sub_point_cloud;
ros::Subscriber sub_point_cloud2;
ros::Subscriber sub_lidar;
ros::Subscriber sub_sonar0;
ros::Subscriber sub_sonar1;
ros::Subscriber sub_sonar2;
sensor_msgs::PointCloud2::Ptr point_cloud_ptr ;
sensor_msgs::PointCloud2::Ptr point_cloud_ptr2;
sensor_msgs::LaserScan::Ptr   laser_scan_ptr  ;
float sonar_reading0;
float sonar_reading1;
float sonar_reading2;
tf::TransformListener* tf_listener;
float global_goal_x = 99999;
float global_goal_y = 99999;

Eigen::Affine3d get_transform_to_basefootprint(std::string link_name)
{
    tf::StampedTransform tf;
    tf_listener->lookupTransform(base_link_name, link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void get_robot_pose(float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void callback_lidar(sensor_msgs::LaserScan::Ptr msg)
{
    laser_scan_ptr = msg;
    no_data_lidar_counter = 0;
}

void callback_point_cloud(sensor_msgs::PointCloud2::Ptr msg)
{
    point_cloud_ptr = msg;
    no_data_cloud_counter = 0;
}

void callback_point_cloud2(sensor_msgs::PointCloud2::Ptr msg)
{
    point_cloud_ptr2 = msg;
    no_data_cloud_counter2 = 0;
}

void callback_sonar0(const sensor_msgs::Range::ConstPtr& msg)
{
    sonar_reading0 = msg->range;
    no_data_sonars_counter = 0;
}

void callback_sonar1(const sensor_msgs::Range::ConstPtr& msg)
{
    sonar_reading1 = msg->range;
}
void callback_sonar2(const sensor_msgs::Range::ConstPtr& msg)
{
    sonar_reading2 = msg->range;
}

void callback_goal_path(const nav_msgs::Path::ConstPtr& msg)
{
     global_goal_x = msg->poses[msg->poses.size() - 1].pose.position.x;
     global_goal_y = msg->poses[msg->poses.size() - 1].pose.position.y;
}

void callbackEnable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "ObsDetector.->Starting obstacle detection using: " << (use_lidar ? "lidar " : "") << (use_sonars?"sonars ":"");
        std::cout << (use_cloud ? "point_cloud" : "") << (use_cloud2 ? "point_cloud2" : "") <<std::endl;
        if(use_cloud )  sub_point_cloud  = nh->subscribe(point_cloud_topic , 1, callback_point_cloud );
        if(use_cloud2)  sub_point_cloud2 = nh->subscribe(point_cloud_topic2, 1, callback_point_cloud2);
        if(use_lidar )  sub_lidar        = nh->subscribe(laser_scan_topic , 1, callback_lidar);
        if(use_sonars)  sub_sonar0       = nh->subscribe(sonars_topic_prefix + "0", 1, callback_sonar0);
        if(use_sonars)  sub_sonar1       = nh->subscribe(sonars_topic_prefix + "1", 1, callback_sonar1);
        if(use_sonars)  sub_sonar2       = nh->subscribe(sonars_topic_prefix + "2", 1, callback_sonar2);
    }
    else
    {
        std::cout << "ObsDetector.->Stopping obstacle detection..." <<std::endl;
        if(use_cloud)   sub_point_cloud .shutdown();
        if(use_cloud2)  sub_point_cloud2.shutdown();
        if(use_lidar)   sub_lidar       .shutdown();
        if(use_sonars)  sub_sonar0      .shutdown();
        if(use_sonars)  sub_sonar1      .shutdown();
        if(use_sonars)  sub_sonar2      .shutdown();
    }
    enable = msg->data;
}

float get_search_distance()
{
    float robot_x, robot_y, robot_a;
    get_robot_pose(robot_x, robot_y, robot_a);
    float dist_to_goal = sqrt(pow(global_goal_x - robot_x, 2) + pow(global_goal_x - robot_x, 2));
    if(dist_to_goal < maxX)
        return dist_to_goal;
    return maxX;
}

bool collisionRiskWithCloud(float& collisionX, float& collisionY)
{
    if(point_cloud_ptr == NULL)
    {
        std::cout << "ObsDetector.->CANNOT GET POINT CLOUD from topic " << point_cloud_topic <<  "!!!" << std::endl;
        return false;
    }
    if((ros::Time::now() - point_cloud_ptr->header.stamp) > ros::Duration(0.5))
        std::cout << "ObsDetector.->POINT CLOUD IS TOO OLD!!! WARNING!!! POSSIBLE COLLISION RISK UNDETECTED!!!" << std::endl;

    //SELECT MAX X ACCORDING TO DISTANCE TO GOAL POINT
    float optimal_x = get_search_distance();
    
    unsigned char* p = (unsigned char*)(&point_cloud_ptr->data[0]);
    int count = 0;
    collisionX = 0;
    collisionY = 0;
    Eigen::Affine3d cam_to_robot = get_transform_to_basefootprint(point_cloud_frame);
    for(size_t i=0; i < point_cloud_ptr->width*point_cloud_ptr->height; i+=cloud_downsampling)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > minX && v.x() < optimal_x && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ)
        {
            count ++;
            collisionX += v.x();
            collisionY += v.y();
        }
        p += cloud_downsampling*point_cloud_ptr->point_step;
    }
    if(debug)
        std::cout<<"ObsDetector.->Cloud Size:"<<point_cloud_ptr->width<<"x"<<point_cloud_ptr->height<<"  Counter: " << count << std::endl;
    if(count > cloud_points_threshold)
    {
        collisionX /= count;
        collisionY /= count;
        return true;
    }
    collisionX = 0;
    collisionY = 0;
    return false;
}

bool collisionRiskWithCloud2(float& collisionX, float& collisionY)
{
    if(point_cloud_ptr2 == NULL)
    {
        std::cout << "ObsDetector.->CANNOT GET POINT CLOUD 2 from topic " << point_cloud_topic2 <<  "!!!" << std::endl;
        return false;
    }
    if((ros::Time::now() - point_cloud_ptr2->header.stamp) > ros::Duration(0.5))
        std::cout << "ObsDetector.->POINT CLOUD 2 IS TOO OLD!!! WARNING!!! POSSIBLE COLLISION RISK UNDETECTED!!!" << std::endl;

    //SELECT MAX X ACCORDING TO DISTANCE TO GOAL POINT
    float optimal_x = get_search_distance();
    
    unsigned char* p = (unsigned char*)(&point_cloud_ptr2->data[0]);
    int count = 0;
    collisionX = 0;
    collisionY = 0;
    Eigen::Affine3d cam_to_robot = get_transform_to_basefootprint(point_cloud_frame2);
    for(size_t i=0; i < point_cloud_ptr2->width*point_cloud_ptr2->height; i+=cloud_downsampling2)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        if(v.x() > minX && v.x() < optimal_x && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ)
        {
            count ++;
            collisionX += v.x();
            collisionY += v.y();
        }
        p += cloud_downsampling2*point_cloud_ptr2->point_step;
    }
    if(debug)
        std::cout<<"ObsDetector.->Cloud Size2:"<<point_cloud_ptr2->width<<"x"<<point_cloud_ptr2->height<<"  Counter: "<<count<<std::endl;
    if(count > cloud_points_threshold2)
    {
        collisionX /= count;
        collisionY /= count;
        return true;
    }
    collisionX = 0;
    collisionY = 0;
    return false;
}

bool collisionRiskWithLidar(float& collisionX, float& collisionY)
{
    if(laser_scan_ptr == NULL)
    {
        std::cout << "ObsDetector.->CANNOT GET LASER SCAN CLOUD from topic " << laser_scan_topic << "!!!" << std::endl;
        return false;
    }
    if((ros::Time::now() - laser_scan_ptr->header.stamp) > ros::Duration(0.5))
        std::cout << "ObsDetector.->LASER SCAN IS TOO OLD!!! WARNING!!! POSSIBLE COLLISION RISK UNDETECTED!!!" << std::endl;

    //SELECT MAX X ACCORDING TO DISTANCE TO GOAL POINT
    float optimal_x = get_search_distance();

    int count = 0;
    collisionX = 0;
    collisionY = 0;
    Eigen::Affine3d lidar_to_robot = get_transform_to_basefootprint(laser_scan_frame);
    for(size_t i=0; i < laser_scan_ptr->ranges.size(); i+=lidar_downsampling)
    {
        float angle = laser_scan_ptr->angle_min + i*laser_scan_ptr->angle_increment;
        Eigen::Vector3d v(laser_scan_ptr->ranges[i]*cos(angle), laser_scan_ptr->ranges[i]*sin(angle), 0);
        v = lidar_to_robot * v;
        if(v.x() > minX && v.x() < optimal_x && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ)
        {
            count ++;
            collisionX += v.x();
            collisionY += v.y();
        }
    }
    if(debug)
        std::cout << "ObsDetector.->LaserScan Size: " << laser_scan_ptr->ranges.size() << "  Counter: " << count << std::endl;
    if(count > lidar_points_threshold)
    {
        collisionX /= count;
        collisionY /= count;
        return true;
    }
    // Obtener promedio de puntos fuera del bounding box
    collisionX = 0;
    collisionY = 0;
    return false;
}

bool collisionRiskWithSonars(float& collisionX, float& collisionY, float sonar_angle_0, float sonar_angle_1, float sonar_angle_2)
{
    //angles of sonar position are just an approximate;
    float x0 = sonar_reading0*cos(sonar_angle_0);
    float y0 = sonar_reading0*sin(sonar_angle_0);
    float x1 = sonar_reading1*cos(sonar_angle_1);
    float y1 = sonar_reading1*sin(sonar_angle_1);
    float x2 = sonar_reading2*cos(sonar_angle_2);
    float y2 = sonar_reading2*sin(sonar_angle_2);

    //SELECT MAX X ACCORDING TO DISTANCE TO GOAL POINT
    float optimal_x = get_search_distance();

    bool collisionRisk = x0 > minX && x0 < optimal_x && y0 > minY && y0 < maxY;
    collisionRisk |= x1 > minX && x1 < optimal_x && y1 > minY && y1 < maxY;
    collisionRisk |= x2 > minX && x2 < optimal_x && y2 > minY && y2 < maxY;
    collisionX = (x0 + x1 + x2)/3;
    collisionY = (y0 + y1 + y2)/3;
    if(debug)
        std::cout << "ObsDetector.->Sonars: "<<sonar_reading0<<"  "<<sonar_reading1<<"  "<<sonar_reading2<<"  "<<(collisionRisk?"Collision!":"") << std::endl;
    return collisionRisk;
}

bool check_collision_risk(float sonar_angle_0, float sonar_angle_1, float sonar_angle_2 ,bool use_lidar, bool use_sonars, bool use_point_cloud, float& collisionX, float& collisionY)
{
    if(current_speed_linear <= 0 && !debug)
    {
        collisionX = 0;
        collisionY = 0;
        return false;
    }
    return (use_lidar && collisionRiskWithLidar(collisionX, collisionY)) ||
        (use_sonars && collisionRiskWithSonars(collisionX, collisionY, sonar_angle_0, sonar_angle_1, sonar_angle_2)) ||
        (use_cloud && collisionRiskWithCloud(collisionX, collisionY)) ||
        (use_cloud2 && collisionRiskWithCloud2(collisionX, collisionY));
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_speed_linear  = msg->linear.x;
    current_speed_angular = msg->angular.z;
}

float get_sonar_angle(const char* frame)
{
    try{
        tf::TransformListener listener;
        tf::StampedTransform transform;
        std::cout << "Waiting for transform to get sonar position on frame: " << frame << std::endl;
        listener.waitForTransform("/base_link", frame, ros::Time(0), ros::Duration(0));
        listener.lookupTransform("/base_link", frame, ros::Time(0), transform);

        tf::Quaternion q;
        q = transform.getRotation();
        float theta = atan2((float)q.z(), (float)q.w()) * 2;

        return theta;
    }
    catch(const std::exception& e){
        return 0.0;
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBSTACLE DETECTOR NODE BY MARCO NEGRETE... " << std::endl;
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle n;
    ros::Rate loop(RATE);
    tf_listener = new tf::TransformListener();
    nh = &n;

    float no_sensor_data_timeout = 0.5;
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
    if(ros::param::has("~cloud_points_threshold"))
        ros::param::get("~cloud_points_threshold",  cloud_points_threshold);
    if(ros::param::has("~cloud_points_threshold2"))
        ros::param::get("~cloud_points_threshold2",  cloud_points_threshold2);
    if(ros::param::has("~cloud_downsampling"))
        ros::param::get("~cloud_downsampling", cloud_downsampling);
    if(ros::param::has("~cloud_downsampling2"))
        ros::param::get("~cloud_downsampling2", cloud_downsampling2);
    if(ros::param::has("~lidar_points_threshold"))
        ros::param::get("~lidar_points_threshold",  lidar_points_threshold);
    if(ros::param::has("~lidar_downsampling"))
        ros::param::get("~lidar_downsampling", lidar_downsampling);
    if(ros::param::has("~obs_detector_debug"))
        ros::param::get("~obs_detector_debug", debug);
    if(ros::param::has("no_sensor_data_timeout"))
        ros::param::get("no_sensor_data_timeout", no_sensor_data_timeout);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);

    std::cout << "ObsDetector.->Starting obstacle detection using: "<< (use_lidar ? "lidar " : "") << (use_sonars ? "sonars " : "");
    std::cout << (use_cloud ? "point_cloud" : "") << (use_cloud2 ? "point_cloud2": "") << std::endl;
    std::cout << "ObsDetector.->Using parameters: min_x=" << minX << "  max_x=" << maxX << "  min_y=" << minY << "  max_y=";
    std::cout << maxY << "  min_z=" << minZ << "  max_z=" << maxZ << std::endl;
    std::cout << "ObsDetector.->Point cloud topic: " << point_cloud_topic << "   Point cloud2 topic: " << point_cloud_topic2;
    std::cout << "   lidar topic name: " << laser_scan_topic << std::endl;
    std::cout << "ObsDetector.->Using parameters: cloud_points_threshold="  << cloud_points_threshold  << "  cloud_downsampling=";
    std::cout << cloud_downsampling  << std::endl;
    std::cout << "ObsDetector.->Using parameters: cloud_points_threshold2=" << cloud_points_threshold2 << "  cloud_downsampling2=";
    std::cout << cloud_downsampling2 << std::endl;
    std::cout << "ObsDetector.->Using parameters: lidar_points_threshold=" << lidar_points_threshold << "  lidar_downsampling=";
    std::cout << lidar_downsampling << std::endl;
    std::cout << "ObsDetector.->Sonars topic prefix: " << sonars_topic_prefix << std::endl;
    std::cout << "ObsDetector.->Base link frame: " << base_link_name << std::endl;

    std::cout << "ObsDetector.->Trying to get first messages from active sensor topics: " << (use_cloud ? point_cloud_topic : "") << "  ";
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
    std::cout << "ObsDetector.->First messages received..." << std::endl;
    if(use_cloud  && ptr_c1   != NULL) point_cloud_frame  = ptr_c1->header.frame_id;
    if(use_cloud2 && ptr_c2   != NULL) point_cloud_frame2 = ptr_c2->header.frame_id;
    if(use_lidar  && ptr_scan != NULL) laser_scan_frame   = ptr_scan->header.frame_id;
    

    std::cout << "ObsDetect.->Waiting for transforms to be available..." << std::endl;
    tf_listener->waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(1000.0));
    std::cout << "MapAugmenter.->Waiting for sensor transforms" << std::endl;
    if (use_cloud ) tf_listener->waitForTransform(base_link_name, point_cloud_frame , ros::Time(0), ros::Duration(1000.0));
    if (use_cloud2) tf_listener->waitForTransform(base_link_name, point_cloud_frame2, ros::Time(0), ros::Duration(1000.0));
    if (use_lidar)  tf_listener->waitForTransform(base_link_name, laser_scan_frame,   ros::Time(0), ros::Duration(1000.0));
    std::cout << "MapAugmenter.->Sensor transforms are now available"<< std::endl;

    float sonar_angle_0 = 0.0;
    float sonar_angle_1 = 0.0;
    float sonar_angle_2 = 0.0;
    sonar_angle_0 = get_sonar_angle("/usound_0_link");
    sonar_angle_1 = get_sonar_angle("/usound_1_link");
    sonar_angle_2 = get_sonar_angle("/usound_2_link");

    std::cout << "ObsDetect.->Sonar 0 angle: " << sonar_angle_0 << std::endl;
    std::cout << "ObsDetect.->Sonar 1 angle: " << sonar_angle_1 << std::endl;
    std::cout << "ObsDetect.->Sonar 2 angle: " << sonar_angle_2 << std::endl;

    ros::Subscriber subEnable = n.subscribe("/navigation/obs_detector/enable", 1, callbackEnable);
    ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1, callback_cmd_vel);
    ros::Subscriber sub_goal_path    = n.subscribe("/simple_move/goal_path", 10, callback_goal_path);
    ros::Publisher pubCollisionRisk  = n.advertise<std_msgs::Bool>("/navigation/obs_detector/collision_risk", 1);
    ros::Publisher pubCollisionPoint = n.advertise<geometry_msgs::PointStamped>("/navigation/obs_detector/collision_point", 1);
    ros::Publisher pubNoSensorData   = n.advertise<std_msgs::Empty>("/navigation/obs_detector/no_sensor_data", 1);

    float collisionX;
    float collisionY;
    std_msgs::Bool msgCollisionRisk;
    geometry_msgs::PointStamped msgCollisionPoint;
    msgCollisionPoint.header.frame_id = base_link_name;
    bool no_lidar_data  = false;
    bool no_cloud_data  = false;
    bool no_cloud2_data = false;
    bool no_sonars_data = false;

    int counter = 0;
    while(ros::ok())
    {
        if(enable)
        {
            counter = 0;
            msgCollisionRisk.data = check_collision_risk(sonar_angle_0, sonar_angle_1, sonar_angle_2, use_lidar, use_sonars, use_cloud, collisionX, collisionY);
            msgCollisionPoint.point.x = collisionX;
            msgCollisionPoint.point.y = collisionY;
            pubCollisionRisk.publish(msgCollisionRisk);
            pubCollisionPoint.publish(msgCollisionPoint);
            if(use_lidar  && (no_lidar_data  = no_data_lidar_counter++  > no_sensor_data_timeout*RATE))
                std::cout << "ObsDetector.->WARNING!!! No lidar data received from topic: " << laser_scan_topic << std::endl;
            if(use_cloud  && (no_cloud_data  = no_data_cloud_counter++  > no_sensor_data_timeout*RATE))
                std::cout << "ObsDetector.->WARNING!!! No cloud data received from topic: " << point_cloud_topic << std::endl;
            if(use_cloud2 && (no_cloud2_data = no_data_cloud_counter2++ > no_sensor_data_timeout*RATE))
                std::cout << "ObsDetector.->WARNING!!! No cloud2 data received from topic: " << point_cloud_topic2 << std::endl;
            if(use_sonars && (no_sonars_data = no_data_sonars_counter++ > no_sensor_data_timeout*RATE))
                std::cout << "ObsDetector.->WARNING!!! No sonars data from topics: " << sonars_topic_prefix << std::endl;
            if((use_lidar && no_lidar_data) || (use_cloud && no_cloud_data) || (use_cloud2 && no_cloud2_data) || (use_sonars && no_sonars_data))
                pubNoSensorData.publish(std_msgs::Empty());
                
        }
        ros::spinOnce();
        loop.sleep();
    }
    delete tf_listener;
}
