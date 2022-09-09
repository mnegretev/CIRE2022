/// @copyright Copyright (C) 2017 Toyota Motor Corporation
#ifndef HSRB_MOVEIT_PLUGINS_HSRB_GRIPPER_COMMAND_CONTROLLER_HANDLE_HPP_
#define HSRB_MOVEIT_PLUGINS_HSRB_GRIPPER_COMMAND_CONTROLLER_HANDLE_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/controller_manager/controller_manager.h>
#include <tf/tf.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>

namespace {
const char* kHandMotorJointName = "hand_motor_joint";
}  // anonymous namespace

// オリジナルのコードの変更量を抑えるためにnamespaceは同じにする
namespace moveit_simple_controller_manager {

class HsrbGripperCommandControllerHandle : public ActionBasedControllerHandleBase {
 protected:
  // どっちのアクションを実行中か
  enum ActionType {
    kNone = 0,
    kFollowJointTrajectory,
    kGripperApplyEffort
  };

 public:
  HsrbGripperCommandControllerHandle(const std::string& name,
                                     const std::string& action_ns,
                                     const std::string& gripper_ns)
      : ActionBasedControllerHandleBase(name),
        executing_action_(kNone) {
    // アクションクライアントを2つ作る
    std::string follow_action_name = name + "/" + action_ns;
    std::string gripper_action_name = name + "/" + gripper_ns;
    follow_action_client_.reset(
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(follow_action_name, true));
    gripper_action_client_.reset(
        new actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction>(gripper_action_name, true));

    if (!follow_action_client_->waitForServer(ros::Duration(15.0))) {
      ROS_ERROR("HsrbGripperCommandControllerHandle: FollowJointTrajectoryAction client not connected.");
      follow_action_client_.reset();
      return;
    }
    if (!gripper_action_client_->waitForServer(ros::Duration(15.0))) {
      ROS_ERROR("HsrbGripperCommandControllerHandle: GripperApplyEffortAction client not connected");
      gripper_action_client_.reset();
      return;
    }
  }

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) {
    if (!isConnected()) {
      return false;
    }

    // hand_motor_jointのインデックスを取得する
    std::size_t hand_index;
    for (hand_index = 0; hand_index < trajectory.joint_trajectory.joint_names.size(); ++hand_index) {
      if (trajectory.joint_trajectory.joint_names[hand_index] == kHandMotorJointName) {
        break;
      }
    }
    if (hand_index >= trajectory.joint_trajectory.joint_names.size()) {
      ROS_WARN("HsrbGripperCommandControllerHandle: '%s' is not included in trajectory.", kHandMotorJointName);
      return false;
    }

    // hand_motor_jointだけのJointTrajectoryを作成する
    // 同時にhand_motor_jointの最小のeffortを取得する
    trajectory_msgs::JointTrajectory hand_trajectory;
    hand_trajectory.header = trajectory.joint_trajectory.header;
    hand_trajectory.joint_names.push_back(kHandMotorJointName);
    double min_effort = 0.0;
    for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
      const trajectory_msgs::JointTrajectoryPoint& point = trajectory.joint_trajectory.points[i];
      if (point.positions.size() > hand_index) {
        trajectory_msgs::JointTrajectoryPoint new_point;
        new_point.positions.push_back(point.positions[hand_index]);
        if (point.velocities.size() > hand_index) {
          new_point.velocities.push_back(point.velocities[hand_index]);
        }
        if (point.accelerations.size() > hand_index) {
          new_point.accelerations.push_back(point.accelerations[hand_index]);
        }
        if (point.effort.size() > hand_index) {
          min_effort = std::min(min_effort, point.effort[hand_index]);
        }
        new_point.time_from_start = point.time_from_start;
        hand_trajectory.points.push_back(new_point);
      }
    }

    // trajectoryのどこか1点でも負のeffortがあったら、Graspを行う
    //
    // MoveItは姿勢のプランニングなので基本的にはeffortは0のままである
    // 一方そのままでは、握りこみの動作等ができないので
    // http://docs.ros.org/jade/api/moveit_msgs/html/msg/Grasp.html
    // のtrajectory_msgs/JointTrajectory grasp_posture
    // にあるように、
    // 最後の握りこみのところだけは、任意のJointTrajectoryを直接与えることができるようになっている。
    // HSRでこの仕様に対応するために、
    // Trajectory内に1点でも負のeffortがあれば、GripperApplyEffortを行うという仕様にしている。
    // (ユーザはposition適当、effortが負のJointTrajectoryをgrasp_postureに与える)
    if (min_effort < -std::numeric_limits<double>::epsilon()) {
      tmc_control_msgs::GripperApplyEffortGoal goal;
      goal.effort = min_effort;
      gripper_action_client_->sendGoal(
          goal, boost::bind(&HsrbGripperCommandControllerHandle::gripperApplyEffortGoalDoneCallback, this, _1, _2));
      executing_action_ = kGripperApplyEffort;
    } else {
      // 特に負のeffortが与えられていなければ、普通のFollowJointTrajectory
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = hand_trajectory;
      follow_action_client_->sendGoal(
          goal, boost::bind(&HsrbGripperCommandControllerHandle::followJointTrajectoryDoneCallback, this, _1, _2));
      executing_action_ = kFollowJointTrajectory;
    }
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  virtual bool cancelExecution() {
    if (!isConnected()) {
      return false;
    }
    if (executing_action_ != kNone) {
      // 動作中がどちらかで止めるアクションを変更する
      if (executing_action_ == kFollowJointTrajectory) {
        ROS_INFO("HsrbGripperCommandControllerHandle: Cancelling execution for FollowJointTrajectoryAction.");
        follow_action_client_->cancelGoal();
      } else if (executing_action_ == kGripperApplyEffort) {
        ROS_INFO("HsrbGripperCommandControllerHandle: Cancelling execution for GripperApplyEffortAction.");
        gripper_action_client_->cancelGoal();
      }
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      executing_action_ = kNone;
    }
    return true;
  }

  virtual bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) {
    if (!isConnected() || executing_action_ == kNone) {
      return false;
    }
    if (executing_action_ == kFollowJointTrajectory) {
      ROS_DEBUG("HsrbGripperCommandControllerHandle: Waiting for completion of FollowJointTrajectoryAction.");
      return follow_action_client_->waitForResult(timeout);
    } else if (executing_action_ == kGripperApplyEffort) {
      ROS_DEBUG("HsrbGripperCommandControllerHandle: Waiting for completion of GripperApplyEffortAction.");
      return gripper_action_client_->waitForResult(timeout);
    }
  }

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus() {
    return last_exec_;
  }

  bool isConnected() const {
    return follow_action_client_ && gripper_action_client_;
  }

  virtual void addJoint(const std::string& name) {
    joints_.push_back(name);
  }

  virtual void getJoints(std::vector<std::string>& joints) {
    joints = joints_;
  }

 protected:
  void finishControllerExecution(const actionlib::SimpleClientGoalState& state) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    else
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    executing_action_ = kNone;
  }

  void followJointTrajectoryDoneCallback(const actionlib::SimpleClientGoalState& state,
                                         const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_DEBUG_STREAM("HsrbGripperCommandControllerHandle: FollowJointTrajectoryAction is done with state "
                     << state.toString()
                     << ": " << state.getText());
    finishControllerExecution(state);
  }

  void gripperApplyEffortGoalDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const tmc_control_msgs::GripperApplyEffortResultConstPtr& result) {
    ROS_DEBUG_STREAM("HsrbGripperCommandControllerHandle: GripperApplyEffortAction is done with state "
                     << state.toString()
                     << ": " << state.getText());
    finishControllerExecution(state);
  }

  // executing action
  ActionType executing_action_;

  // execution status
  moveit_controller_manager::ExecutionStatus last_exec_;

  // the joints controlled by this controller
  std::vector<std::string> joints_;

  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > follow_action_client_;
  boost::shared_ptr<actionlib::SimpleActionClient<tmc_control_msgs::GripperApplyEffortAction> > gripper_action_client_;
};

}  // namespace moveit_simple_controller_manager

#endif  // HSRB_MOVEIT_PLUGINS_HSRB_GRIPPER_COMMAND_CONTROLLER_HANDLE_HPP_
