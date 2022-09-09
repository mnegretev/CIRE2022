/// @copyright Copyright (C) 2017 Toyota Motor Corporation
/// @brief Controller handle for HSR-B omnibase

#ifndef HSRB_MOVEIT_CONTROLLER_MANAGER_FOLLOW_OMNIBASE_TRAJECTORY_CONTROLLER_HANDLE_HPP_
#define HSRB_MOVEIT_CONTROLLER_MANAGER_FOLLOW_OMNIBASE_TRAJECTORY_CONTROLLER_HANDLE_HPP_

#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <tf/tf.h>

// オリジナルのコードの変更量を抑えるためにnamespaceは同じにする
namespace moveit_simple_controller_manager {

class FollowOmniBaseTrajectoryControllerHandle
    : public moveit_simple_controller_manager::ActionBasedControllerHandle<
                 control_msgs::FollowJointTrajectoryAction> {
 public:
  FollowOmniBaseTrajectoryControllerHandle(const std::string& name,
                                           const std::string& action_ns)
    : ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, action_ns) {}

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory) {
    if (!controller_action_client_) {
      return false;
    }
    if (trajectory.multi_dof_joint_trajectory.joint_names.size() != 1) {
      ROS_WARN("FollowOmniBaseTrajectoryController: %s requires a multi-dof trajectory.",
               name_.c_str());
      return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header = trajectory.joint_trajectory.header;
    goal.trajectory.joint_names.push_back("odom_x");
    goal.trajectory.joint_names.push_back("odom_y");
    goal.trajectory.joint_names.push_back("odom_t");
    for (uint32_t i = 0; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i) {
      trajectory_msgs::JointTrajectoryPoint trajectory_point;
      if (trajectory.multi_dof_joint_trajectory.points[i].transforms.size() != 1) {
        ROS_WARN("FollowOmniBaseTrajectoryController: The size of transforms in trajectory should be 1");
        return false;
      }

      trajectory_point.time_from_start =
          trajectory.multi_dof_joint_trajectory.points[i].time_from_start;
      trajectory_point.positions.push_back(
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.x);
      trajectory_point.positions.push_back(
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.y);

      tf::Quaternion q(
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.x,
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y,
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z,
          trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.w);
      double roll;
      double pitch;
      double yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // -pi - piであればそのままいれても問題ない
      trajectory_point.positions.push_back(yaw);
      goal.trajectory.points.push_back(trajectory_point);
    }

    controller_action_client_->sendGoal(
        goal, boost::bind(&FollowOmniBaseTrajectoryControllerHandle::DoneCallback, this, _1, _2));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

 protected:
  void DoneCallback(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    finishControllerExecution(state);
  }
};

}  // namespace moveit_simple_controller_manager

#endif  // HSRB_MOVEIT_CONTROLLER_MANAGER_FOLLOW_OMNIBASE_TRAJECTORY_CONTROLLER_HANDLE_HPP_
