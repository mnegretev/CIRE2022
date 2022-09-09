/// @copyright Copyright (C) 2017 Toyota Motor Corporation
#include "hsrb_moveit_kinematics.hpp"
#include <string>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <hsrb_analytic_ik/hsrb_ik_solver.hpp>
#include <hsrb_analytic_ik/hsrc_ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_kinematics_model/tarp3_wrapper.hpp>

// register HSRBKinematicsPlugin as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(hsrb_moveit_kinematics::HSRBKinematicsPlugin, kinematics::KinematicsBase);

namespace hsrb_moveit_kinematics {

using tmc_robot_kinematics_model::IKSolver;
using tmc_manipulation_types::JointState;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::kSuccess;

HSRBKinematicsPlugin::HSRBKinematicsPlugin() : active_(false) {}

bool HSRBKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                      const std::string& base_frame, const std::string& tip_frame,
                                      double search_discretization) {
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
  ros::NodeHandle nh;
  std::string urdf_string;
  if (!nh.getParam(robot_description, urdf_string)) {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "failed to read : " << robot_description);
    return false;
  }
  robot_ = static_cast<IRobotKinematicsModel::Ptr>(new Tarp3Wrapper(urdf_string));

  dimension_ = 8;

  joint_names_.push_back("world_joint");
  joint_names_.push_back("arm_lift_joint");
  joint_names_.push_back("arm_flex_joint");
  joint_names_.push_back("arm_roll_joint");
  joint_names_.push_back("wrist_flex_joint");
  joint_names_.push_back("wrist_roll_joint");

  link_names_.push_back("base_footprint");
  link_names_.push_back("arm_lift_link");
  link_names_.push_back("arm_flex_link");
  link_names_.push_back("arm_roll_link");
  link_names_.push_back("wrist_flex_link");
  link_names_.push_back("wrist_roll_link");

  // 利用関節名
  use_joints_.push_back("arm_lift_joint");
  use_joints_.push_back("arm_flex_joint");
  use_joints_.push_back("arm_roll_joint");
  use_joints_.push_back("wrist_flex_joint");
  use_joints_.push_back("wrist_roll_joint");

  // デフォルトのウェイト
  weights_.push_back(10.0);
  weights_.push_back(10.0);
  weights_.push_back(1.0);
  weights_.push_back(10.0);
  weights_.push_back(1.0);
  weights_.push_back(1.0);
  weights_.push_back(1.0);
  weights_.push_back(1.0);

  // IKで用いるJointのデフォルトの重みをrosparamから読み込む
  // kinematics.yamlで設定できるように、読み込む先の仕様をKinemaicsプラグインの仕様に合わせる
  // https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_ros/planning/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp#L301
  //
  // 以下の順で読み込む
  // 1. ~/joint_group_name/kinematics_solver_weights
  // 2. /robot_description_kinematics/joint_group_name/kinematics_solver_weights
  ros::NodeHandle private_node("~");
  std::string base_param_name = group_name;

  std:: string ksolver_robot_name;
  if (private_node.searchParam(robot_description + "_kinematics/robot_name", ksolver_robot_name)) {
    private_node.getParam(robot_description + "_kinematics/robot_name", ksolver_robot_name);
  }
  if (ksolver_robot_name == "hsrc") {
    solver_.reset(new hsrb_analytic_ik::HsrcIKSolver(IKSolver::Ptr()));
  } else {
    solver_.reset(new hsrb_analytic_ik::HsrbIKSolver(IKSolver::Ptr()));
  }

  std::string ksolver_param_name;
  bool found = private_node.searchParam(base_param_name + "/kinematics_solver", ksolver_param_name);
  if (!found || !private_node.hasParam(ksolver_param_name)) {
    base_param_name = robot_description + "_kinematics/" + group_name;
  }

  std::string ksolver_weights_param_name;
  if (private_node.searchParam(base_param_name + "/kinematics_solver_weights", ksolver_weights_param_name)) {
    if (private_node.hasParam(ksolver_weights_param_name)) {
      if (private_node.getParam(ksolver_weights_param_name, weights_)) {
        if (weights_.size() != 8) {
          ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "kinematics_solver_weights size is not 8");
          return false;
        }
      } else {
        ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "kinematics_solver_weights type is wrong");
        return false;
      }
    }
  }

  if (tip_frame != "hand_palm_link") {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "tip_frame should be 'hand_palm_link'");
    return false;
  }

  if (base_frame != "odom") {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "base_frame should be 'odom'");
    return false;
  }

  active_ = true;
  ROS_DEBUG_NAMED("hsrb_moveit_kinematics", "TMC HSRB solver initialized");

  return true;
}

bool HSRBKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const {
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool HSRBKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                         std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                         const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}


bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const {
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool HSRBKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const std::vector<double>& consistency_limits,
                                            const kinematics::KinematicsQueryOptions& options) const {
  error_code.val = error_code.NO_IK_SOLUTION;
  if (!active_) {
    ROS_ERROR_NAMED("hsrb_moveit_kinematics", "kinematics not active");
    return false;
  }

  if (ik_seed_state.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "Seed state must have size " << dimension_ << " instead of size "
                           << ik_seed_state.size());
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "Consistency limits be empty or must have size "
                           << dimension_ << " instead of size " << consistency_limits.size());
    return false;
  }

  solution.resize(dimension_);

  // 手先目標位置の変換
  Eigen::Affine3d ref_origin_to_end;
  tf::poseMsgToEigen(ik_pose, ref_origin_to_end);
  ROS_DEBUG_STREAM_NAMED("hsrb_moveit_kinematics", "ik pose: \n" << ik_pose);

  // 数値IKの関節の初期値
  Eigen::VectorXd init_angle;
  init_angle.resize(dimension_ - 3);
  for (int i = 0; i < dimension_ - 3; ++i) {
    init_angle[i] = ik_seed_state[i + 3];
  }

  // 台車の自由度をplanar拘束(x,y,θに設定
  IKRequest req(tmc_manipulation_types::kPlanar);
  req.frame_name = "hand_palm_link";
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.initial_angle.name = use_joints_;
  req.use_joints = use_joints_;
  // 関節の重み。ただし後ろ3つは台車の分。
  req.weight = Eigen::Map<const Eigen::VectorXd>(&weights_[0], weights_.size());
  req.ref_origin_to_end = ref_origin_to_end;
  req.initial_angle.position = init_angle;
  req.origin_to_base =                                                  \
      Eigen::Translation3d(ik_seed_state[0], ik_seed_state[1], 0)
      * Eigen::AngleAxisd(ik_seed_state[2], Eigen::Vector3d::UnitZ());

  JointState js_solution;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  tmc_robot_kinematics_model::IKResult result;

  result = solver_->Solve(req, js_solution, origin_to_base_solution, origin_to_hand_result);

  if (result == kSuccess) {
    ROS_DEBUG_STREAM_NAMED("hsrb_moveit_kinematics", "ik solved\n");
    ROS_DEBUG_STREAM_NAMED("hsrb_moveit_kinematics", "joints:\n" << js_solution.position);
    ROS_DEBUG_STREAM_NAMED("hsrb_moveit_kinematics",
                           "origin_to_base_solution:\n" << origin_to_base_solution.matrix());
    ROS_DEBUG_STREAM_NAMED("hsrb_moveit_kinematics", "result:\n" << origin_to_hand_result.matrix());

    Eigen::Vector3d euler_vector = origin_to_base_solution.rotation().eulerAngles(0, 1, 2);

    solution[0] = origin_to_base_solution.translation().x();
    solution[1] = origin_to_base_solution.translation().y();
    solution[2] = euler_vector(2);

    for (int i = 3; i < dimension_; ++i) {
      solution[i] = js_solution.position[i - 3];
    }

    error_code.val = error_code.SUCCESS;
    if (!solution_callback.empty()) {
      solution_callback(ik_pose, solution, error_code);
    }
  }
  return error_code.val == error_code.SUCCESS;
}

bool HSRBKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                         const std::vector<double>& joint_angles,
                                         std::vector<geometry_msgs::Pose>& poses) const {
  if (joint_angles.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "joint_angles size is not " << dimension_);
    return false;
  }
  // 関節角の設定
  JointState joint_state;
  joint_state.name = use_joints_;
  joint_state.position.resize(dimension_ - 3);
  for (int i = 0; i < dimension_ - 3; ++i) {
    joint_state.position[i] = joint_angles[i + 3];
  }
  robot_->SetNamedAngle(joint_state);
  // ベース位置の設定
  Eigen::Affine3d origin_to_base;
  origin_to_base =                                                      \
      Eigen::Translation3d(joint_angles[0], joint_angles[1], 0)
      * Eigen::AngleAxisd(joint_angles[2], Eigen::Vector3d::UnitZ());
  robot_->SetRobotTransform(origin_to_base);
  poses.clear();
  for (std::size_t i = 0; i < link_names.size(); ++i) {
    Eigen::Affine3d origin_to_link;
    geometry_msgs::Pose pose;
    try {
      origin_to_link = robot_->GetObjectTransform(link_names[i]);
    } catch (const tmc_robot_kinematics_model::Tarp3Error& err) {
      ROS_ERROR_STREAM_NAMED("hsrb_moveit_kinematics", "getPositionFK failed to find '" << link_names[i] << "'");
      return false;
    }
    tf::poseEigenToMsg(origin_to_link, pose);
    poses.push_back(pose);
  }
  return true;
}

const std::vector<std::string>& HSRBKinematicsPlugin::getJointNames() const { return joint_names_; }

const std::vector<std::string>& HSRBKinematicsPlugin::getLinkNames() const { return link_names_; }

bool HSRBKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const {
  return true;
}

}  // namespace hsrb_moveit_kinematics
