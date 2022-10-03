 #include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "tf/transform_listener.h"
#include "actionlib_msgs/GoalStatus.h"

#define RATE 30

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 11
#define SM_GOAL_POSE_ACCEL 1
#define SM_GOAL_POSE_CRUISE 2
#define SM_GOAL_POSE_DECCEL 3
#define SM_GOAL_POSE_CORRECT_ANGLE 4
#define SM_GOAL_POSE_FINISH 10
#define SM_GOAL_POSE_FAILED 12
#define SM_GOAL_PATH_FIRST_POINT 49
#define SM_GOAL_PATH_ACCEL 5
#define SM_GOAL_PATH_CRUISE 6
#define SM_GOAL_PATH_DECCEL 7
#define SM_GOAL_PATH_FINISH 8
#define SM_GOAL_PATH_FAILED 81
#define SM_COLLISION_RISK 9

float goal_distance  = 0;
float goal_angle     = 0;
bool  new_pose       = false;
bool  new_path       = false;
bool  collision_risk = false;
nav_msgs::Path goal_path;
bool stop = false;
std::string base_link_name = "base_footprint";

void callback_general_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SimpleMove.->General Stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
}

void callback_navigation_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SimpleMove.->Navigation Stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
}

void callback_simple_move_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "SimpleMove.->Simple move stop signal received" << std::endl;
    stop     = true;
    new_pose = false;
    new_path = false;
}

void callback_goal_dist(const std_msgs::Float32::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data << std::endl;     
    goal_distance = msg->data;
    goal_angle    = 0;
    new_pose = true;
    new_path = false;
}

void callback_goal_dist_angle(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New move received: goal dist= " << msg->data[0] << " and goal angle= " << msg->data[1] << std::endl;
    goal_distance = msg->data[0];
    goal_angle    = msg->data[1];
    new_pose = true;
    new_path = false;
}

void callback_goal_path(const nav_msgs::Path::ConstPtr& msg)
{
    std::cout << "SimpleMove.->New path received with " << msg->poses.size() << " points with id=" << msg->header.seq << std::endl;
    if (msg->poses.size() <= 0)
    {
        new_pose = false;
        new_path = false;   
    }else{
        goal_path = *msg;
        new_pose = false;
        new_path = true;
    }
}

void callback_collision_risk(const std_msgs::Bool::ConstPtr& msg)
{
    collision_risk = msg->data;
}

geometry_msgs::Twist calculate_speeds(float robot_x, float robot_y, float robot_t, float goal_x, float goal_y,
                                      float min_linear_speed, float max_linear_speed, float angular_speed, float alpha, float beta, bool backwards)
{
    float angle_error = 0;
    if(backwards) angle_error = atan2(robot_y - goal_y, robot_x - goal_x) - robot_t;
    else angle_error = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;

    if(backwards) max_linear_speed *= -1;
    geometry_msgs::Twist result;
    result.linear.x  = max_linear_speed  * exp(-(angle_error * angle_error) / alpha);
    if(fabs(result.linear.x) < min_linear_speed)
        result.linear.x = 0;
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;
}

geometry_msgs::Twist calculate_speeds(float robot_angle, float goal_angle, float angular_speed, float beta)
{
    float angle_error = goal_angle - robot_angle;
    if(angle_error >   M_PI) angle_error -= 2*M_PI;
    if(angle_error <= -M_PI) angle_error += 2*M_PI;

    geometry_msgs::Twist result;
    result.linear.x  = 0;
    result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);
    return result;
}

void get_robot_position_wrt_map(tf::TransformListener& tf_listener, float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("map", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void get_robot_position_wrt_odom(tf::TransformListener& tf_listener, float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("odom", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}

void get_goal_position_wrt_odom(float goal_distance, float goal_angle, tf::TransformListener& tf_listener,
                                float& goal_x, float& goal_y, float& goal_t)
{
    tf::StampedTransform transform;
    tf_listener.lookupTransform("odom", base_link_name, ros::Time(0), transform);
    float robot_x = transform.getOrigin().x();
    float robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    float robot_t = atan2((float)q.z(), (float)q.w()) * 2;

    goal_x = robot_x + goal_distance * cos(robot_t + goal_angle);
    goal_y = robot_y + goal_distance * sin(robot_t + goal_angle);
    goal_t = robot_t + goal_angle;
    if(goal_t >   M_PI) goal_t -= 2*M_PI;
    if(goal_t <= -M_PI) goal_t += 2*M_PI;
}

float get_path_total_distance(nav_msgs::Path& path)
{
    float dist = 0;
    for(size_t i=1; i < path.poses.size(); i++)
        dist += sqrt(pow(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,2) + pow(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,2));
    return dist;
}

void find_nearest_path_point(float& robot_x, float& robot_y, float& robot_t, float& goal_x, float& goal_y,
                             int& next_pose_idx, tf::TransformListener& tf_listener)
{
    get_robot_position_wrt_map(tf_listener, robot_x, robot_y, robot_t);
    next_pose_idx = 0;
    float d1 = sqrt(fabs(goal_path.poses[next_pose_idx].pose.position.x - robot_x) +
                    fabs(goal_path.poses[next_pose_idx].pose.position.y - robot_y));
    float d2 = sqrt(fabs(goal_path.poses[next_pose_idx+1].pose.position.x - robot_x) +
                    fabs(goal_path.poses[next_pose_idx+1].pose.position.y - robot_y));
    while(d2 < d1)
    {
        next_pose_idx++;
        d1 = sqrt(fabs(goal_path.poses[next_pose_idx].pose.position.x - robot_x) +
                  fabs(goal_path.poses[next_pose_idx].pose.position.y - robot_y));
        d2 = sqrt(fabs(goal_path.poses[next_pose_idx+1].pose.position.x - robot_x) +
                  fabs(goal_path.poses[next_pose_idx+1].pose.position.y - robot_y));
    }//This is done to avoid looping overall path points to find the nearest one
    goal_x = goal_path.poses[next_pose_idx+1].pose.position.x;
    goal_y = goal_path.poses[next_pose_idx+1].pose.position.y;
}

void get_next_goal_from_path(float robot_x, float robot_y, float robot_t, float& goal_x, float& goal_y, int& next_pose_idx)
{
    if(next_pose_idx >= goal_path.poses.size()) next_pose_idx = goal_path.poses.size() - 1;
    float error = 0;
    do
    {
        goal_x = goal_path.poses[next_pose_idx].pose.position.x;
        goal_y = goal_path.poses[next_pose_idx].pose.position.y;
        error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
    }while(error < 0.25 && ++next_pose_idx < goal_path.poses.size());
}

std_msgs::Float32MultiArray get_next_goal_head_angles(float robot_x, float robot_y, float robot_t, int next_pose_idx)
{
    std_msgs::Float32MultiArray msg;
    int idx = next_pose_idx + 5 >=  goal_path.poses.size() - 1 ? goal_path.poses.size() - 1 : next_pose_idx + 5;
    float goal_x = goal_path.poses[idx].pose.position.x;
    float goal_y = goal_path.poses[idx].pose.position.y;
    float a = atan2(goal_y - robot_y, goal_x - robot_x) - robot_t;
    if(a >   M_PI) a -= 2*M_PI;
    if(a <= -M_PI) a += 2*M_PI;
    msg.data.push_back(a);
    msg.data.push_back(-1.0);
    return msg;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SIMPLE MOVE NODE BY MARCO NEGRETE..." << std::endl;
    ros::init(argc, argv, "simple_move");
    ros::NodeHandle n("~");
    
    ros::Subscriber sub_goalDistance     = n.subscribe("/simple_move/goal_dist"      , 1, callback_goal_dist);                     
    ros::Subscriber sub_goalDistAngle    = n.subscribe("/simple_move/goal_dist_angle", 1, callback_goal_dist_angle);          
    ros::Subscriber sub_goalPath         = n.subscribe("/simple_move/goal_path"      , 1, callback_goal_path);                     
    ros::Subscriber sub_generalStop      = n.subscribe("/stop", 1, callback_general_stop);
    ros::Subscriber sub_navCtrlStop      = n.subscribe("/navigation/stop",  1, callback_navigation_stop);
    ros::Subscriber sub_navSimpleMvStop  = n.subscribe("/simple_move/stop", 1, callback_simple_move_stop);               
    ros::Subscriber sub_gollisionRisk    = n.subscribe("/navigation/obs_detector/collision_risk", 10, callback_collision_risk);
    tf::TransformListener tf_listener;
    ros::Rate loop(RATE);

    float max_linear_speed  = 0.3;
    float min_linear_speed  = 0.05;
    float max_angular_speed = 1.0;
    float alpha = 0.6548;
    float beta = 0.2;
    float linear_acceleration = 0.1;
    float fine_dist_tolerance = 0.03;
    float coarse_dist_tolerance = 0.2;
    std::string  cmd_vel_name="/cmd_vel";


    float angle_tolerance = 0.05;
    bool  move_head = true;

    if(ros::param::has("~max_linear_speed"))
        ros::param::get("~max_linear_speed", max_linear_speed);
    if(ros::param::has("~min_linear_speed"))
        ros::param::get("~min_linear_speed", min_linear_speed);
    if(ros::param::has("~max_angular_speed"))
        ros::param::get("~max_angular_speed",max_angular_speed);
    if(ros::param::has("~control_alpha"))
        ros::param::get("~control_alpha", alpha);
    if(ros::param::has("~control_beta"))
        ros::param::get("~control_beta", beta);
    if(ros::param::has("~linear_acceleration"))
        ros::param::get("~linear_acceleration", linear_acceleration);
    if(ros::param::has("~fine_dist_tolerance"))
        ros::param::get("~fine_dist_tolerance", fine_dist_tolerance);
    if(ros::param::has("~coarse_dist_tolerance"))
        ros::param::get("~coarse_dist_tolerance", coarse_dist_tolerance);
    if(ros::param::has("~angle_tolerance"))
        ros::param::get("~angle_tolerance", angle_tolerance);
    if(ros::param::has("~move_head"))
        ros::param::get("~move_head", move_head);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);
    if(ros::param::has("~cmd_vel_name"))
        ros::param::get("~cmd_vel_name", cmd_vel_name);


    std::cout << "SimpleMove.->Control parameters: min_linear=" << min_linear_speed << " max_linear=" << max_linear_speed;
    std::cout << "  linear_accel=" << linear_acceleration << "  max_angular=" << max_angular_speed << std::endl;
    std::cout << "SimpleMove.->alpha=" << alpha << "  beta=" << beta << "  fine_dist_tol=" << fine_dist_tolerance;
    std::cout << "  coarse_dist_tol=" << coarse_dist_tolerance << "  angle_tol=" << angle_tolerance << std::endl;
    std::cout << "SimpleMove.->Base link name: " << base_link_name << std::endl;
    std::cout<<"cmd_vel_name: "<<cmd_vel_name<<std::endl;

    std::cout << "SimpleMove.->Waiting for odometry and localization transforms..." << std::endl;
    tf_listener.waitForTransform("map",  base_link_name, ros::Time(0), ros::Duration(1000.0));
    tf_listener.waitForTransform("odom", base_link_name, ros::Time(0), ros::Duration(1000.0));

    ros::Publisher pub_goal_reached     = n.advertise<actionlib_msgs::GoalStatus>("/simple_move/goal_reached", 1); 
    ros::Publisher pub_cmd_vel          = n.advertise<geometry_msgs::Twist>(cmd_vel_name, 1);
    ros::Publisher pub_head_goal_pose   = n.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);

    actionlib_msgs::GoalStatus msg_goal_reached;
    int state = SM_INIT;
    float current_linear_speed = 0;
    float robot_x = 0;
    float robot_y = 0;
    float robot_t = 0;
    float goal_x = 0;
    float goal_y = 0;
    float goal_t = 0;
    float global_goal_x = 0;
    float global_goal_y = 0;
    int next_pose_idx = 0;
    float temp_k = 0;
    int attempts = 0;
    float error = 0;
    while(ros::ok())
    {
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::GoalStatus::ABORTED;
            pub_cmd_vel.publish(geometry_msgs::Twist());
            pub_goal_reached.publish(msg_goal_reached);
        }
        if(new_pose || new_path)
            state = SM_WAITING_FOR_TASK;
        
        
        switch(state)
        {
        case SM_INIT:
            std::cout << "SimpleMove.->Low level control ready. Waiting for new goal. " << std::endl;
            state = SM_WAITING_FOR_TASK;
            break;

            
        case SM_WAITING_FOR_TASK:
            if(new_pose)
            {
                get_goal_position_wrt_odom(goal_distance, goal_angle, tf_listener, goal_x, goal_y, goal_t);
                state = SM_GOAL_POSE_ACCEL;
                new_pose = false;
                msg_goal_reached.goal_id.id = "-1";
                attempts = (int)((fabs(goal_distance)*5)/max_linear_speed*RATE + fabs(goal_angle*5)/max_angular_speed*RATE + RATE*5);
            }
            if(new_path)
            {
                state = SM_GOAL_PATH_FIRST_POINT;
                new_path = false;
                global_goal_x = goal_path.poses[goal_path.poses.size() - 1].pose.position.x;
                global_goal_y = goal_path.poses[goal_path.poses.size() - 1].pose.position.y;
                std::stringstream ss;
                ss << goal_path.header.seq;
                ss >> msg_goal_reached.goal_id.id;
                attempts = (int)(get_path_total_distance(goal_path)/max_linear_speed*4*RATE + 5*RATE);
            }
            break;

            
        case SM_GOAL_POSE_ACCEL:
            get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
            error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(error < fine_dist_tolerance)
                state = SM_GOAL_POSE_CORRECT_ANGLE;
            else if(error < current_linear_speed*current_linear_speed/linear_acceleration)
            {
                state = SM_GOAL_POSE_DECCEL;
                temp_k = current_linear_speed/sqrt(error);
            }
            else if(current_linear_speed >= max_linear_speed)
            {
                current_linear_speed = max_linear_speed;
                state = SM_GOAL_POSE_CRUISE;
            }
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_ACCEL." << std::endl;
            }
            pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, goal_distance < 0));
            current_linear_speed += linear_acceleration/RATE;
            break;

            
        case SM_GOAL_POSE_CRUISE:
            get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
            error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(error < fine_dist_tolerance)
                state = SM_GOAL_POSE_CORRECT_ANGLE;
            else if(error < current_linear_speed*current_linear_speed/linear_acceleration)
            {
                temp_k = current_linear_speed/sqrt(error);
                state = SM_GOAL_POSE_DECCEL;
            }
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CRUISE." << std::endl;
            }
            pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, goal_distance < 0));
            break;

            
        case SM_GOAL_POSE_DECCEL:
            get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
            error = sqrt((goal_x - robot_x)*(goal_x - robot_x) + (goal_y - robot_y)*(goal_y - robot_y));
            if(error < fine_dist_tolerance)
                    state = SM_GOAL_POSE_CORRECT_ANGLE;
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_DECCEL." << std::endl;
            }
            current_linear_speed = temp_k * sqrt(error);
            if(current_linear_speed < min_linear_speed) current_linear_speed = min_linear_speed;
            pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, goal_distance < 0));
            break;


        case SM_GOAL_POSE_CORRECT_ANGLE:
            get_robot_position_wrt_odom(tf_listener, robot_x, robot_y, robot_t);
            error = (goal_t - robot_t);
            if (error > M_PI) error-=2*M_PI;
            if (error <= -M_PI) error+=2*M_PI;
            error = fabs(error);
            if(error < angle_tolerance)
                state = SM_GOAL_POSE_FINISH;
            pub_cmd_vel.publish(calculate_speeds(robot_t, goal_t, max_angular_speed, beta));
            if(--attempts <= 0)
            {
                state = SM_GOAL_POSE_FAILED;
                std::cout << "SimpleMove.->Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CORRECT_ANGLE." << std::endl;
            }   
            break;


        case SM_GOAL_POSE_FINISH:
            std::cout << "SimpleMove.->Successful move with dist=" << goal_distance << " angle=" << goal_angle << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::GoalStatus::SUCCEEDED;
            pub_goal_reached.publish(msg_goal_reached);
            pub_cmd_vel.publish(geometry_msgs::Twist());
            current_linear_speed = 0;
            break;


        case SM_GOAL_POSE_FAILED:
            std::cout << "SimpleMove.->FAILED move with dist=" << goal_distance << " angle=" << goal_angle << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::GoalStatus::ABORTED;
            pub_goal_reached.publish(msg_goal_reached);
            pub_cmd_vel.publish(geometry_msgs::Twist());
            current_linear_speed = 0;
            break;

            
        case SM_GOAL_PATH_FIRST_POINT:
            find_nearest_path_point(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx, tf_listener);
            state = SM_GOAL_PATH_ACCEL;
            break;

            
        case SM_GOAL_PATH_ACCEL:
            if(collision_risk)
            {
                pub_cmd_vel.publish(geometry_msgs::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else{
                get_robot_position_wrt_map(tf_listener, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx);
                error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                else if(error < current_linear_speed*current_linear_speed/linear_acceleration)
                {
                    state = SM_GOAL_PATH_DECCEL;
                    temp_k = current_linear_speed/sqrt(error);
                }
                else if(current_linear_speed >= max_linear_speed)
                {
                    current_linear_speed = max_linear_speed;
                    state = SM_GOAL_PATH_CRUISE;
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_ACCEL."<<std::endl;
                }
                pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false));
                if(move_head) pub_head_goal_pose.publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
                current_linear_speed += linear_acceleration/RATE;
            }
            break;


        case SM_GOAL_PATH_CRUISE:
            if(collision_risk)
            {
                pub_cmd_vel.publish(geometry_msgs::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else
            {
                get_robot_position_wrt_map(tf_listener, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx);
                error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                else if(error < current_linear_speed*current_linear_speed/linear_acceleration)
                {
                    temp_k = current_linear_speed/sqrt(error);
                    state = SM_GOAL_PATH_DECCEL;
                }
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_CRUISE."<<std::endl;
                }
                pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false));
                if(move_head) pub_head_goal_pose.publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
            }
            break;

            
        case SM_GOAL_PATH_DECCEL:
            if(collision_risk)
            {
                pub_cmd_vel.publish(geometry_msgs::Twist());
                std::cout << "SimpleMove.->WARNING! Collision risk detected!!!!!" << std::endl;
                state = SM_GOAL_PATH_FAILED;
            }
            else
            {
                get_robot_position_wrt_map(tf_listener, robot_x, robot_y, robot_t);
                get_next_goal_from_path(robot_x, robot_y, robot_t, goal_x, goal_y, next_pose_idx);
                error = sqrt((global_goal_x - robot_x)*(global_goal_x - robot_x) + (global_goal_y - robot_y)*(global_goal_y - robot_y));
                if(error < coarse_dist_tolerance)
                    state = SM_GOAL_PATH_FINISH;
                if(--attempts <= 0)
                {
                    state = SM_GOAL_PATH_FAILED;
                    std::cout<<"SimpleMove.->Timeout exceeded while trying to reach goal path. Current state:GOAL_PATH_DECCEL."<<std::endl;
                }
                current_linear_speed = temp_k * sqrt(error);
                if(current_linear_speed < min_linear_speed) current_linear_speed = min_linear_speed;
                pub_cmd_vel.publish(calculate_speeds(robot_x, robot_y, robot_t, goal_x, goal_y, min_linear_speed, current_linear_speed,
                                                     max_angular_speed, alpha, beta, false));
                if(move_head) pub_head_goal_pose.publish(get_next_goal_head_angles(robot_x, robot_y, robot_t, next_pose_idx));
            }
            break;

            
        case SM_GOAL_PATH_FINISH:
            std::cout << "SimpleMove.->Path succesfully followed." << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::GoalStatus::SUCCEEDED;
            pub_goal_reached.publish(msg_goal_reached);
            pub_cmd_vel.publish(geometry_msgs::Twist());
            current_linear_speed = 0;
            break;


        case SM_GOAL_PATH_FAILED:
            std::cout << "SimpleMove.->FAILED path traking." << std::endl;
            state = SM_INIT;
            msg_goal_reached.status = actionlib_msgs::GoalStatus::ABORTED;
            pub_goal_reached.publish(msg_goal_reached);
            pub_cmd_vel.publish(geometry_msgs::Twist());
            current_linear_speed = 0;
            break;

        default:
            std::cout << "SimpleMove.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE. :'(" << std::endl;
            return -1;
        }
        ros::spinOnce();
        loop.sleep();
    }
}
