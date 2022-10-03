#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "actionlib_msgs/GoalStatus.h"
#include "tf/transform_listener.h"

#define RATE 10

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_CHECK_IF_INSIDE_OBSTACLES 19
#define SM_WAITING_FOR_MOVE_BACKWARDS 18
#define SM_CHECK_IF_OBSTACLES 21
#define SM_WAIT_FOR_NO_OBSTACLES 22
#define SM_ENABLE_OBS_DETECT 23
#define SM_WAIT_FOR_OBS_DETECT 24
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_STOP_RECEIVED 51
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_FINAL    17

bool stop = false;
bool collision_risk = false;
bool  patience = true;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool new_global_goal = false;
geometry_msgs::Pose global_goal;
std::string base_link_name = "base_footprint";

void callback_general_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->General Stop signal received" << std::endl;
    stop = true;
}

void callback_navigation_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->Navigation Stop signal received" << std::endl;
    stop = true;
}

void callback_simple_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    global_goal = msg->pose;
    new_global_goal = true;
}

void callback_collision_risk(const std_msgs::Bool::ConstPtr& msg){
    collision_risk = msg->data;
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}

void callback_set_patience(const std_msgs::Bool::ConstPtr& msg)
{
    std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False") << std::endl;
    patience = msg->data;
}

bool plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y, ros::ServiceClient& clt, nav_msgs::Path& path)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = robot_x;
    srv.request.start.pose.position.y = robot_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!clt.call(srv))
        return false;
    path = srv.response.plan;
    return true;
}

void get_robot_position(tf::TransformListener& listener, float& robot_x, float& robot_y, float& robot_a){
    tf::StampedTransform tf;
    tf::Quaternion q;
    listener.lookupTransform("map", base_link_name, ros::Time(0), tf);
    robot_x = tf.getOrigin().x();
    robot_y = tf.getOrigin().y();
    q = tf.getRotation();
    robot_a = atan2((float)q.z(), (float)q.w()) * 2;
}

int publish_status(int status, int id, std::string text, ros::Publisher& pub)
{
    actionlib_msgs::GoalStatus msg;
    std::stringstream ss;
    ss << id;
    msg.status = status;
    msg.text   = text;
    ss >> msg.goal_id.id;
    msg.goal_id.stamp = ros::Time::now();
    pub.publish(msg);
    return status;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING MVN_PLN NODE BY MARCO NEGRETE..." << std::endl;
    ros::init(argc, argv, "mvn_pln");
    ros::NodeHandle n;
    tf::TransformListener listener;
    
    float proximity_criterion = 2.0;
    if(ros::param::has("~patience"))
        ros::param::get("~patience", patience);
    if(ros::param::has("~proximity_criterion"))
        ros::param::get("~proximity_criterion", proximity_criterion);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);

    std::cout << "MvnPln.->Patience: " << (patience?"True":"False") << "  Proximity criterion: " << proximity_criterion;
    std::cout << "  base link name: " << base_link_name << std::endl;
    std::cout << "MvnPln.->Waiting for localization transform..." << std::endl;
    listener.waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(1000.0));
    std::cout << "MvnPln.->Localization transform is ready ..." << std::endl;
    std::cout << "MvnPln.->Waiting for path planner to be ready..." << std::endl;
    ros::service::waitForService("/path_planner/plan_path_with_static"   , ros::Duration(1000));
    ros::service::waitForService("/path_planner/plan_path_with_augmented", ros::Duration(10000));
    std::cout << "MvnPln.->Path planner is ready." << std::endl;
    std::cout << "mvnPln.->Waiting for map augmenter to be ready..." << std::endl;
    ros::service::waitForService("/map_augmenter/get_augmented_map"     , ros::Duration(1000));
    ros::service::waitForService("/map_augmenter/get_augmented_cost_map", ros::Duration(1000));
    ros::service::waitForService("/map_augmenter/are_there_obstacles"   , ros::Duration(1000));
    std::cout << "MvnPln.->Map Augmenter is ready..." << std::endl;
    
    ros::Subscriber sub_generalStop        = n.subscribe("/stop", 10, callback_general_stop);
    ros::Subscriber sub_navCtrlStop        = n.subscribe("/navigation/stop", 10, callback_navigation_stop);               
    ros::Subscriber sub_simple_goal        = n.subscribe("/nav_control/goal", 10, callback_simple_goal);
    ros::Subscriber sub_collision_risk     = n.subscribe("/navigation/obs_detector/collision_risk", 10, callback_collision_risk);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_set_patience       = n.subscribe("/navigation/set_patience", 10, callback_set_patience);
    ros::Publisher pub_obs_detector_enable = n.advertise<std_msgs::Bool   >("/navigation/obs_detector/enable", 1);
    ros::Publisher pub_goal_path           = n.advertise<nav_msgs::Path   >("/simple_move/goal_path", 1);
    ros::Publisher pub_goal_dist_angle     = n.advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 1);
    ros::Publisher pub_status              = n.advertise<actionlib_msgs::GoalStatus>("/navigation/status", 10);
    ros::Publisher pub_simple_move_stop    = n.advertise<std_msgs::Empty>("/simple_move/stop", 1);

    ros::ServiceClient clt_plan_path       = n.serviceClient<nav_msgs::GetPlan>("/path_planner/plan_path_with_augmented");
    ros::ServiceClient clt_are_there_obs   = n.serviceClient<std_srvs::Trigger>("/map_augmenter/are_there_obstacles");
    ros::ServiceClient clt_is_in_obstacles = n.serviceClient<std_srvs::Trigger>("/map_augmenter/is_inside_obstacles");
    
    ros::Rate loop(RATE);
    ros::Rate slow_loop(1);

    float robot_x = 0;
    float robot_y = 0;
    float robot_a = 0;
    float error = 0;

    int  state = SM_INIT;
    int  simple_move_sequencer = -1;
    int  goal_id = -1;
    int  current_status = 0;
    bool near_goal_sent = false;
    std_msgs::Bool msg_bool;
    std_srvs::Trigger srv_check_obstacles;
    nav_msgs::Path path;
    std_msgs::Float32MultiArray msg_goal_dist_angle;
    msg_goal_dist_angle.data.resize(2);

    while(ros::ok())
    {
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                current_status=publish_status(actionlib_msgs::GoalStatus::ABORTED,goal_id,"Stop signal received. Task cancelled",pub_status);
        }
        if(new_global_goal)
            state = SM_WAITING_FOR_TASK;
        
        
        switch(state)
        {
        case SM_INIT:
            std::cout << "MvnPln.->MVN PLN READY. Waiting for new goal. " << std::endl;
            state = SM_WAITING_FOR_TASK;
            break;

        case SM_WAITING_FOR_TASK:
            if(new_global_goal)
            {
                new_global_goal = false;
                state = SM_CALCULATE_PATH;
                if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cancelling current movement.", pub_status);
                goal_id++;
                std::cout << "MvnPln.->New goal received. Current task goal_id: " << goal_id << std::endl;
                current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Starting new movement task", pub_status);
                near_goal_sent = false;
            }
            break;

            
        case SM_CALCULATE_PATH:
            get_robot_position(listener, robot_x, robot_y, robot_a);
            if(!plan_path_from_augmented_map(robot_x, robot_y, global_goal.position.x, global_goal.position.y, clt_plan_path, path))
            {
                std::cout<<"MvnPln.->Cannot calc path to "<< global_goal.position.x <<" "<<global_goal.position.y << std::endl;
                pub_simple_move_stop.publish(std_msgs::Empty());
                if(!patience)
                    state = SM_CHECK_IF_INSIDE_OBSTACLES;
                else
                    state = SM_CHECK_IF_OBSTACLES;
            }
            else
                state = SM_ENABLE_OBS_DETECT;
            break;

        case SM_CHECK_IF_INSIDE_OBSTACLES:
            std::cout << "MvnPln.->Checking if robot is inside an obstacle..." << std::endl;
            clt_is_in_obstacles.call(srv_check_obstacles);
            if(srv_check_obstacles.response.success)
            {
                std::cout << "MvnPln.->Robot is inside an obstacle. Moving backwards..." << std::endl;
                msg_goal_dist_angle.data[0] = -0.25;
                msg_goal_dist_angle.data[1] = 0;
                pub_goal_dist_angle.publish(msg_goal_dist_angle);
                state = SM_WAITING_FOR_MOVE_BACKWARDS;
            }
            else
            {
                current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cannot calc path from start to goal", pub_status);
                state = SM_INIT;
            }
            break;

        case SM_WAITING_FOR_MOVE_BACKWARDS:
            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Moved backwards succesfully." << std::endl;
            }
            else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Simple move reported move aborted. " << std::endl;
            }
            state = SM_CALCULATE_PATH;
            break;
            
        case SM_CHECK_IF_OBSTACLES:
            if(!clt_are_there_obs.call(srv_check_obstacles) || !srv_check_obstacles.response.success)
            {
                std::cout << "MvnPln.->There are no temporal obstacles. Announcing failure." << std::endl;
                current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                "Cannot calculate path from start to goal point", pub_status);
                state = SM_INIT;
            }
            else
            {
                std::cout << "MvnPln.->Temporal obstacles detected. Waiting for them to move." << std::endl;
                current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id,
                                                "Waiting for temporal obstacles to move", pub_status);
                state = SM_WAIT_FOR_NO_OBSTACLES;
            }
            break;


        case SM_WAIT_FOR_NO_OBSTACLES:
            if(!clt_are_there_obs.call(srv_check_obstacles))
            {
                std::cout << "MvnPln.->Cannot call service for checking temporal obstacles. Announcing failure." << std::endl;
                current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                "Cannot calculate path from start to goal point", pub_status);
                state = SM_INIT;
            }
            else if(!srv_check_obstacles.response.success)
            {
                std::cout << "MvnPln.->Temporal obstacles removed. " << std::endl;
                state = SM_CALCULATE_PATH;
            }
            else
                slow_loop.sleep();
            break;

            
        case SM_ENABLE_OBS_DETECT:
            msg_bool.data = true;
            pub_obs_detector_enable.publish(msg_bool);
            std::cout << "MvnPln.->Obstacle detector enable flag sent. Waiting for obs detector to be enabled..." << std::endl;
            state = SM_WAIT_FOR_OBS_DETECT;
            break;
            

        case SM_WAIT_FOR_OBS_DETECT:
            ros::topic::waitForMessage<std_msgs::Bool>("/navigation/obs_detector/collision_risk", ros::Duration(100.0));
            std::cout << "MvnPln.->Obstacle detector is now available." << std::endl;
            state = SM_START_MOVE_PATH;
            break;

            
        case SM_START_MOVE_PATH:
            std::cout << "MvnPln. Starting path following " << std::endl;
            collision_risk = false;
            simple_move_sequencer++;
            path.header.seq = simple_move_sequencer;
            pub_goal_path.publish(path);
            simple_move_goal_status.status = 0;
            state = SM_WAIT_FOR_MOVE_FINISHED;
            break;


        case SM_WAIT_FOR_MOVE_FINISHED:
            get_robot_position(listener, robot_x, robot_y, robot_a);
            error = sqrt(pow(global_goal.position.x - robot_x, 2) + pow(global_goal.position.y - robot_y, 2));
            if(error < proximity_criterion && !near_goal_sent)
            {
                near_goal_sent = true;
                std::cout << "MvnPln.->Error less than proximity criterion. Sending near goal point status." << std::endl;
                publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Near goal point", pub_status);
            }
            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == simple_move_sequencer)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Path followed succesfully. " << std::endl;
                msg_bool.data = false;
                pub_obs_detector_enable.publish(msg_bool);
                state = SM_CORRECT_FINAL_ANGLE;
            }
            else if(collision_risk)
            {
                std::cout << "MvnPln.->COLLISION RISK DETECTED before goal is reached." << std::endl;
                collision_risk = false;
                state = SM_CALCULATE_PATH;
            }
            else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Simple move reported path aborted. Trying again..." << std::endl;
                state = SM_CALCULATE_PATH;
            }
            break;
            

        case SM_CORRECT_FINAL_ANGLE:
            std::cout << "MvnPln.->Correcting final angle." << std::endl;
            get_robot_position(listener, robot_x, robot_y, robot_a);
            error = atan2(global_goal.orientation.z, global_goal.orientation.w)*2 - robot_a;
            if(error  >  M_PI) error -= 2*M_PI;
            if(error <= -M_PI) error += 2*M_PI;
            msg_goal_dist_angle.data[0] = 0;
            msg_goal_dist_angle.data[1] = error;
            pub_goal_dist_angle.publish(msg_goal_dist_angle);
            state = SM_WAIT_FOR_ANGLE_CORRECTED;
            break;

                
        case SM_WAIT_FOR_ANGLE_CORRECTED:
            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Final angle corrected succesfully." << std::endl;
                state = SM_FINAL ;
            }
            else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
            {
                simple_move_goal_status.status = 0;
                std::cout << "MvnPln.->Simple move reported move aborted. Trying again..." << std::endl;
                state = SM_CALCULATE_PATH;
            }
            break;
            
                
        case SM_FINAL:
            std::cout << "MvnPln.->TASK FINISHED." << std::endl;
            current_status = publish_status(actionlib_msgs::GoalStatus::SUCCEEDED, goal_id, "Global goal point reached", pub_status);
            state = SM_INIT;
            break;

            
        default:
            std::cout<<"MvnPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE. Please contact your dealer."<<std::endl;
            
        }
        ros::spinOnce();
        loop.sleep();
    }
}
