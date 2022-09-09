/// @copyright Copyright (C) 2017 Toyota Motor Corporation

#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../src/hsrb_moveit_kinematics.hpp"

namespace hsrb_moveit_kinematics {

// initializeのテスト
TEST(HSRBKinematicsPlugin, initialize) {
  {
    // 特に何も設定していないくても成功する
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("/robot_description", "group", "odom", "hand_palm_link", 0.0));
  }
  // 以下はhsrb_moveit_kinematics-test.test内のコメント参照
  {
    HSRBKinematicsPlugin p;
    EXPECT_FALSE(p.initialize("/robot_description", "test_fail_1", "odom", "hand_palm_link", 0.0));
  }
  {
    HSRBKinematicsPlugin p;
    EXPECT_FALSE(p.initialize("/robot_description", "test_fail_2", "odom", "hand_palm_link", 0.0));
  }
  {
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("/robot_description", "test_success_1", "odom", "hand_palm_link", 0.0));
  }
  {
    HSRBKinematicsPlugin p;
    EXPECT_FALSE(p.initialize("robot_description", "test_fail_3", "odom", "hand_palm_link", 0.0));
  }
  {
    HSRBKinematicsPlugin p;
    EXPECT_FALSE(p.initialize("robot_description", "test_fail_4", "odom", "hand_palm_link", 0.0));
  }
  {
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("robot_description", "test_success_2", "odom", "hand_palm_link", 0.0));
  }
}

// getLinkNamesのテスト
TEST(HSRBKinematicsPlugin, getLinkNames) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
  const std::vector<std::string>& link_names = p.getLinkNames();
  EXPECT_EQ(link_names.size(), 6);
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "base_footprint"));
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "arm_lift_link"));
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "arm_flex_link"));
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "arm_roll_link"));
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "wrist_flex_link"));
  EXPECT_NE(link_names.end(), std::find(link_names.begin(), link_names.end(), "wrist_roll_link"));
}

// getJointNamesのテスト
TEST(HSRBKinematicsPlugin, getJointNames) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
  const std::vector<std::string>& joint_names = p.getJointNames();
  EXPECT_EQ(joint_names.size(), 6);
  EXPECT_NE(joint_names.end(), std::find(joint_names.begin(), joint_names.end(), "world_joint"));
  EXPECT_NE(joint_names.end(), std::find(joint_names.begin(), joint_names.end(), "arm_lift_joint"));
  EXPECT_NE(joint_names.end(), std::find(joint_names.begin(), joint_names.end(), "arm_flex_joint"));
  EXPECT_NE(joint_names.end(), std::find(joint_names.begin(), joint_names.end(), "wrist_flex_joint"));
  EXPECT_NE(joint_names.end(), std::find(joint_names.begin(), joint_names.end(), "wrist_roll_joint"));
}

// supportsGroupのテスト
TEST(HSRBKinematicsPlugin, supportsGroup) {
  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
  EXPECT_TRUE(p.supportsGroup(NULL, NULL));
}

// searchPositionIK
TEST(HSRBKinematicsPlugin, searchPositionIK) {
  geometry_msgs::Pose ik_pose;
  std::vector<double> ik_seed_state;
  double timeout;
  std::vector<double> solution;
  kinematics::KinematicsBase::IKCallbackFn solution_callback;
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<double> consistency_limits;
  kinematics::KinematicsQueryOptions options;
  {
    // activeでない
    HSRBKinematicsPlugin p;
    EXPECT_FALSE(p.initialize("robot_description", "test_fail_4", "odom", "hand_palm_link", 0.0));
    EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                    consistency_limits, options));
  }
  {
    // dimenstionが無効
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
    EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                    consistency_limits, options));
  }
  {
    // consistency_limitsのサイズが無効
    HSRBKinematicsPlugin p;
    ik_seed_state.resize(8);
    consistency_limits.resize(2);
    EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
    EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                    consistency_limits, options));
  }
  {
    // 解がでない場合
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
    ik_seed_state.resize(8);
    consistency_limits.clear();
    ik_seed_state[0] = 0.0;
    ik_seed_state[1] = 0.0;
    ik_seed_state[2] = 0.0;
    ik_seed_state[3] = 0.0;
    ik_seed_state[4] = 0.0;
    ik_seed_state[5] = 0.0;
    ik_seed_state[6] = 0.0;
    ik_seed_state[7] = 0.0;
    ik_pose.position.x = 0.0;
    ik_pose.position.y = 0.0;
    ik_pose.position.z = 3.0;
    ik_pose.orientation.x = 0.0;
    ik_pose.orientation.y = 0.0;
    ik_pose.orientation.z = 0.0;
    ik_pose.orientation.w = 1.0;
    EXPECT_FALSE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                    consistency_limits, options));
  }
  {
    // デフォルトの重みでIKを解く
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));
    ik_seed_state.resize(8);
    ik_seed_state[0] = 0.0;
    ik_seed_state[1] = 0.0;
    ik_seed_state[2] = 0.0;
    ik_seed_state[3] = 0.0;
    ik_seed_state[4] = 0.0;
    ik_seed_state[5] = 0.0;
    ik_seed_state[6] = 0.0;
    ik_seed_state[7] = 0.0;
    ik_pose.position.x = 1.0;
    ik_pose.position.y = 0.0;
    ik_pose.position.z = 1.0;
    ik_pose.orientation.x = 0.0;
    ik_pose.orientation.y = 0.0;
    ik_pose.orientation.z = 1.0;
    ik_pose.orientation.w = 0.0;
    EXPECT_TRUE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                   consistency_limits, options));
    // 表示させて答えを作った
    // for(int i = 0; i < 8; ++i) std::cerr << solution[i] << std::endl;
    // 0.838948
    // -0.0726981
    // -0.0358925
    // 0.174505
    // -0.00100738
    // -0
    // 0.00100738
    // 0.0358925
    ASSERT_EQ(8, solution.size());
    EXPECT_NEAR(0.838948,    solution[0], 1e-6);
    EXPECT_NEAR(-0.0726981,  solution[1], 1e-6);
    EXPECT_NEAR(-0.0358925,  solution[2], 1e-6);
    EXPECT_NEAR(0.174505,    solution[3], 1e-6);
    EXPECT_NEAR(-0.00100738, solution[4], 1e-6);
    EXPECT_NEAR(-0,          solution[5], 1e-6);
    EXPECT_NEAR(0.00100738,  solution[6], 1e-6);
    EXPECT_NEAR(0.0358925,   solution[7], 1e-6);
  }
  {
    // 重みを変えた状態でIKを解く
    HSRBKinematicsPlugin p;
    EXPECT_TRUE(p.initialize("robot_description", "weight_test", "odom", "hand_palm_link", 0.0));
    EXPECT_TRUE(p.searchPositionIK(ik_pose, ik_seed_state, 0, solution, solution_callback, error_code,
                                   consistency_limits, options));
    // 表示させて答えを作った
    // for(int i = 0; i < 8; ++i) std::cerr << solution[i] << std::endl;
    // 0.794193
    // -0.0712208
    // -0.0352144
    // 0.178132
    // -0.131457
    // -0
    // 0.131457
    // 0.0352144
    ASSERT_EQ(8, solution.size());
    EXPECT_NEAR(0.794193,    solution[0], 1e-6);
    EXPECT_NEAR(-0.0712208,  solution[1], 1e-6);
    EXPECT_NEAR(-0.0352144,  solution[2], 1e-6);
    EXPECT_NEAR(0.178132,    solution[3], 1e-6);
    EXPECT_NEAR(-0.131457,   solution[4], 1e-6);
    EXPECT_NEAR(-0,          solution[5], 1e-6);
    EXPECT_NEAR(0.131457,    solution[6], 1e-6);
    EXPECT_NEAR(0.0352144,   solution[7], 1e-6);
  }
}

// getPositionFK
TEST(HSRBKinematicsPlugin, getPositionIK) {
  std::vector<std::string> link_names;
  std::vector<double> joint_angles;
  std::vector<geometry_msgs::Pose> poses;

  HSRBKinematicsPlugin p;
  EXPECT_TRUE(p.initialize("robot_description", "group", "odom", "hand_palm_link", 0.0));

  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  joint_angles.push_back(0.0);
  // 関節角が8個ないとFKは失敗
  EXPECT_FALSE(p.getPositionFK(link_names, joint_angles, poses));

  joint_angles.push_back(0.0);
  // link_namesを与えなくてもFKは成功
  EXPECT_TRUE(p.getPositionFK(link_names, joint_angles, poses));

  link_names.push_back("hand_palm_link");
  // hand_palm_linkのFK
  EXPECT_TRUE(p.getPositionFK(link_names, joint_angles, poses));
  ASSERT_EQ(poses.size(), 1);

  // 答えは表示させて作った
  EXPECT_NEAR(poses[0].position.x, 0.158, 1e-6);
  EXPECT_NEAR(poses[0].position.y, 0.078, 1e-6);
  EXPECT_NEAR(poses[0].position.z, 0.8255, 1e-6);
  EXPECT_NEAR(poses[0].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.y, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.z, -1, 1e-6);
  EXPECT_NEAR(poses[0].orientation.w, 0, 1e-6);

  joint_angles[0] = 1.0;
  // base_footprintもFK
  link_names.push_back("base_footprint");
  EXPECT_TRUE(p.getPositionFK(link_names, joint_angles, poses));
  ASSERT_EQ(poses.size(), 2);

  EXPECT_NEAR(poses[0].position.x, 1.158, 1e-6);
  EXPECT_NEAR(poses[0].position.y, 0.078, 1e-6);
  EXPECT_NEAR(poses[0].position.z, 0.8255, 1e-6);
  EXPECT_NEAR(poses[0].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.y, 0, 1e-6);
  EXPECT_NEAR(poses[0].orientation.z, -1, 1e-6);
  EXPECT_NEAR(poses[0].orientation.w, 0, 1e-6);

  EXPECT_NEAR(poses[1].position.x, 1, 1e-6);
  EXPECT_NEAR(poses[1].position.y, 0, 1e-6);
  EXPECT_NEAR(poses[1].position.z, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.x, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.y, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.z, 0, 1e-6);
  EXPECT_NEAR(poses[1].orientation.w, 1, 1e-6);

  // 存在しないリンクだと失敗
  link_names.clear();
  link_names.push_back("dummy");
  // hand_palm_linkのFK
  EXPECT_FALSE(p.getPositionFK(link_names, joint_angles, poses));
}

}  // namespace hsrb_moveit_kinematics

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_hsrb_moveit_kinematics");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
