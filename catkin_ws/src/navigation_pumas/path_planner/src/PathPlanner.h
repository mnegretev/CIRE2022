#include <queue>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"

class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    static bool AStar(nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cost_map,
                      geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose, bool diagonal_paths, nav_msgs::Path& resultPath);
    static nav_msgs::Path SmoothPath(nav_msgs::Path& path, float weight_data = 0.1, float weight_smooth = 0.9, float tolerance = 0.00001);
};

class Node
{
public:
    Node();
    ~Node();

    int   index;           //The index of the corresponding cell in the occupancy grid.
    float g_value;        //The accumulated distance of this node.
    float f_value;         //The f-value, used only in the A* algorithm.
    bool  in_open_list;    //A value indicating whether this node is in the open list or not.
    bool  in_closed_list;  //A value indicating whether this node is in the closed list or not.
    Node* parent;          //A pointer to the parent of this node.
};

class CompareByDistance
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->g_value > n2->g_value; }
};

class CompareByFValue
{
public:
    bool operator()(Node* n1, Node* n2) { return n1->f_value > n2->f_value; }
};
