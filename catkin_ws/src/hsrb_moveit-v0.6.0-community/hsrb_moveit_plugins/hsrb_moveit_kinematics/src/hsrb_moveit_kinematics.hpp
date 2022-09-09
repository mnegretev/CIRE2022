/// @copyright Copyright (C) 2017 Toyota Motor Corporation
#ifndef HSRB_MOVEIT_PLUGINS_HSRB_MOVEIT_KINEMATICS_HPP_
#define HSRB_MOVEIT_PLUGINS_HSRB_MOVEIT_KINEMATICS_HPP_
#include <string>
#include <vector>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>
#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_kinematics_model/tarp3_wrapper.hpp>

using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::Tarp3Wrapper;

/** @brief Namespace for the HSRBKinematics*/
namespace hsrb_moveit_kinematics {
class HSRBKinematicsPlugin : public kinematics::KinematicsBase {
 public:
  /**
   *  @brief Plugin-able interface to the TMC HSRB kinematics
   */
  HSRBKinematicsPlugin();

  /**
   *  @brief Specifies if the solver is active or not
   *  @return True if the solver is active, false otherwise.
   */
  bool isActive();

  virtual bool getPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const;

  /**
   * @brief  Initialization function for the kinematics
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_frame, const std::string& tip_frame, double search_discretization);


  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  bool timedOut(const ros::WallTime& start_time, double duration) const;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const;

  virtual bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const;

 protected:
  bool active_;
  IKSolver::Ptr solver_;
  std::vector<std::string> link_names_;
  std::vector<std::string> joint_names_;
  IRobotKinematicsModel::Ptr robot_;
  tmc_manipulation_types::NameSeq use_joints_;
  std::vector<double> weights_;
  std::size_t dimension_;
};
}  // namespace hsrb_moveit_kinematics

#endif  // HSRB_MOVEIT_PLUGINS_HSRB_MOVEIT_KINEMATICS_HPP_
