/// @copyright Copyright (C) 2017 Toyota Motor Corporation
#ifndef HSRB_MOVEIT_PLUGINS_MOVEIT_FAKE_MULTIDOF_CONTROLLERS_HPP_
#define HSRB_MOVEIT_PLUGINS_MOVEIT_FAKE_MULTIDOF_CONTROLLERS_

#include <limits>
#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <tf2_msgs/TFMessage.h>
#include <urdf_parser/urdf_parser.h>
#include "../thirdparty/moveit_fake_controller_manager/src/moveit_fake_controllers.h"

const char* const kRobotDescription = "robot_description";

// オリジナルのコードの変更量を抑えるためにnamespaceは同じにする
namespace moveit_fake_controller_manager {

// ロボットモデルを毎回パラメータサーバから読み込んでパースするのは時間がかかるので、
// シングルトンでキャッシュする
class RobotModelSingleton {
 public:
  static const robot_model::RobotModelPtr& getInstance() {
    static robot_model::RobotModelPtr robot_model;
    if (!robot_model) {
      robot_model_loader::RobotModelLoader robot_model_loader(kRobotDescription);
      robot_model = robot_model_loader.getModel();
    }
    return robot_model;
  }
  RobotModelSingleton() {}
 private:
  RobotModelSingleton(RobotModelSingleton const&);
  void operator=(RobotModelSingleton const&);
};

// MultiDOFな関節をフェイク制御し状態(TF)を発行するコントローラのベース
// 実際はTFを発行し続けるスレッドをもっていて、
// 継承クラスからupdateTransformsを呼び出すことで発行するTFを更新する
class BaseFakeMultiDOFController {
 public:
  explicit BaseFakeMultiDOFController(const std::vector<std::string> &joints) : tf_rate_(10), cancel_(false) {
    // TF発行用のtf2_msgs::TFMessageをあらかじめ作る
    //
    // 渡されたjointsで
    // ロボットモデル内からplannarとfloatingのジョイントを探す
    // 一致したものがあったら、あらかじめ、parent/childを取得しておく
    // SRDFの仮想ジョイントも検索対象にする
    const std::vector<moveit::core::JointModel*>& joint_models = RobotModelSingleton::getInstance()->getJointModels();
    const srdf::ModelConstSharedPtr& srdf = RobotModelSingleton::getInstance()->getSRDF();
    const std::vector<srdf::Model::VirtualJoint>& virtual_joints = srdf->getVirtualJoints();
    for (std::size_t i = 0; i < joints.size(); ++i) {
      geometry_msgs::TransformStamped t;
      t.transform.rotation.w = 1.0;
      for (std::size_t j = 0; j < virtual_joints.size(); ++j) {
        const srdf::Model::VirtualJoint& joint = virtual_joints[j];
        if (joints[i] == joint.name_) {
          if (joint.type_ == "planar" || joint.type_ == "floating") {
            t.header.frame_id = joint.parent_frame_;
            t.child_frame_id = joint.child_link_;
            tf_.transforms.push_back(t);
            // 名前で更新先を引き当てられるようにマップにいれておく
            tf_map_[joints[i]] = &(tf_.transforms.back().transform);
            break;
          } else {
            throw std::runtime_error("joint <" + joints[i] + "> type is not multi DOF");
          }
        }
      }
      // VirtualJoint内から見つかったら終了
      if (tf_map_.find(joints[i]) != tf_map_.end())
        continue;
      for (std::size_t j = 0; j < joint_models.size(); ++j) {
        const moveit::core::JointModel* joint = joint_models[j];
        if (joints[i] == joint->getName()) {
          if (joint->getTypeName() == "Planar" || joint->getTypeName() == "Floating") {
            t.header.frame_id = joint->getParentLinkModel()->getName();
            t.child_frame_id = joint->getChildLinkModel()->getName();
            tf_.transforms.push_back(t);
            // 名前で更新先を引き当てられるようにマップにいれておく
            tf_map_[joints[i]] = &(tf_.transforms.back().transform);
            break;
          } else {
            throw std::runtime_error("joint <" + joints[i] + "> type is not multi DOF");
          }
        }
      }
      // 見つからなかったらエラー
      if (tf_map_.find(joints[i]) == tf_map_.end())
        throw std::runtime_error("failed to find joint <" + joints[i] + ">");
    }
    thread_ = boost::thread(boost::bind(&BaseFakeMultiDOFController::publishTransforms, this));
  }
  virtual ~BaseFakeMultiDOFController() {
    cancel_ = true;
    thread_.join();
  }

 protected:
  // 発行するtransformsをsensor_msgs::MultiDOFJointStateで更新する
  void updateTransforms(const sensor_msgs::MultiDOFJointState &js) {
    // 更新中はlock
    boost::mutex::scoped_lock lock(mutex_);
    for (std::size_t i = 0; i < js.joint_names.size(); ++i) {
      // 名前で更新先をひきあてて代入
      std::map<std::string, geometry_msgs::Transform*>::iterator it = tf_map_.find(js.joint_names[i]);
      if (it != tf_map_.end()) {
        *(it->second) = js.transforms[i];
      }
    }
  }
  ros::Publisher dummy_;

 private:
  // TFを発行しつづけるスレッド
  void publishTransforms() {
    ros::NodeHandle nh;
    ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 100);
    while (!cancel_) {
      ros::Time t = ros::Time::now();
      for (std::size_t i = 0; i < tf_.transforms.size(); ++i) {
        tf_.transforms[i].header.stamp = t;
      }
      // 発行中はlock
      {
        boost::mutex::scoped_lock lock(mutex_);
        tf_pub.publish(tf_);
      }
      tf_rate_.sleep();
    }
  }
  tf2_msgs::TFMessage tf_;
  std::map<std::string, geometry_msgs::Transform*> tf_map_;
  boost::thread thread_;
  boost::mutex mutex_;
  ros::WallRate tf_rate_;
  bool cancel_;
};

// 入力のMultiDOFJointTrajectoryの最後の点に一気に動かすコントローラ
// LastPointContollerと
// BaseFakeMultiDOFControllerの多重継承
class MultiDOFLastPointController : public LastPointController, BaseFakeMultiDOFController {
 public:
  MultiDOFLastPointController(const std::string& name, const std::vector<std::string>& joints)
      : LastPointController(name, joints, dummy_),
        BaseFakeMultiDOFController(joints) {}
  ~MultiDOFLastPointController() {}

  // LastPointController::sendTrajectoryとほぼ同じ
  // https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_plugins/moveit_fake_controller_manager/src/moveit_fake_controllers.cpp#L77
  // (型が違うだけなのでtemplate化できるが元のコードにあまり手をいれたくないので実施していない)
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& t) {
    ROS_INFO("Fake execution of trajectory");
    if (t.multi_dof_joint_trajectory.points.empty())
      return true;

    sensor_msgs::MultiDOFJointState js;
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& last = t.multi_dof_joint_trajectory.points.back();
    js.joint_names = t.multi_dof_joint_trajectory.joint_names;
    js.transforms = last.transforms;
    updateTransforms(js);

    return true;
  }
};

// 入力のMultiDOFJointTrajectoryの経由点のみを経過時間に合わせて動かしていくコントローラ
// ViaPointControllerと
// BaseFakeMultiDOFControllerの多重継承
class MultiDOFViaPointController : public ViaPointController, BaseFakeMultiDOFController {
 public:
  MultiDOFViaPointController(const std::string& name, const std::vector<std::string>& joints)
      : ViaPointController(name, joints, dummy_),
        BaseFakeMultiDOFController(joints) {}
  ~MultiDOFViaPointController() {}

 protected:
  // ViaPointController::execTrajectoryとほぼ同じ
  // https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_plugins/moveit_fake_controller_manager/src/moveit_fake_controllers.cpp#L163
  // (型が違うだけなのでtemplate化できるが元のコードにあまり手をいれたくないので実施していない)
  void execTrajectory(const moveit_msgs::RobotTrajectory& t) {
    ROS_INFO("Fake execution of trajectory");
    sensor_msgs::MultiDOFJointState js;
    js.joint_names = t.joint_trajectory.joint_names;
    ros::Time startTime = ros::Time::now();

    for (std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator
             via = t.multi_dof_joint_trajectory.points.begin(),
             end = t.multi_dof_joint_trajectory.points.end();
         !cancelled() && via != end; ++via) {
      js.transforms = via->transforms;
      js.twist = via->velocities;

      ros::Duration waitTime = via->time_from_start - (ros::Time::now() - startTime);
      if (waitTime.toSec() > std::numeric_limits<float>::epsilon()) {
        ROS_DEBUG("Fake execution: waiting %0.1fs for next via point, %ld remaining", waitTime.toSec(), end - via);
        waitTime.sleep();
      }
      updateTransforms(js);
    }
    ROS_DEBUG("Fake execution of trajectory: done");
  }
};

namespace {
void interpolate(sensor_msgs::MultiDOFJointState& js, const trajectory_msgs::MultiDOFJointTrajectoryPoint& prev,
                 const trajectory_msgs::MultiDOFJointTrajectoryPoint& next, const ros::Duration& elapsed) {
  double duration = (next.time_from_start - prev.time_from_start).toSec();
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed - prev.time_from_start).toSec() / duration;

  js.transforms.resize(prev.transforms.size());
  for (std::size_t i = 0, end = prev.transforms.size(); i < end; ++i) {
    Eigen::Vector3d prevt, nextt;
    Eigen::Quaterniond prevr, nextr;
    tf::vectorMsgToEigen(prev.transforms[i].translation, prevt);
    tf::vectorMsgToEigen(next.transforms[i].translation, nextt);
    tf::quaternionMsgToEigen(prev.transforms[i].rotation, prevr);
    tf::quaternionMsgToEigen(next.transforms[i].rotation, nextr);
    tf::vectorEigenToMsg(prevt + alpha * (nextt - prevt), js.transforms[i].translation);
    tf::quaternionEigenToMsg(prevr.slerp(alpha, nextr), js.transforms[i].rotation);
  }
}
}  // anonymous namespace

// 入力のMultiDOFJointTrajectoryを経過時間に合わせて補間させながら動かしていくコントローラ
// InterpolatingControllerと
// BaseFakeMultiDOFControllerの多重継承
class MultiDOFInterpolatingController : public InterpolatingController, BaseFakeMultiDOFController {
 public:
  MultiDOFInterpolatingController(const std::string& name, const std::vector<std::string>& joints)
      : InterpolatingController(name, joints, dummy_),
        BaseFakeMultiDOFController(joints) {}
  ~MultiDOFInterpolatingController() {}

 protected:
  // InterpolatingController::execTrajectoryとほぼ同じ
  // https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_plugins/moveit_fake_controller_manager/src/moveit_fake_controllers.cpp#L224
  // (型が違うだけなのでtemplate化できるが元のコードにあまり手をいれたくないので実施していない)
  virtual void execTrajectory(const moveit_msgs::RobotTrajectory& t) {
    ROS_INFO("Fake execution of trajectory");
    if (t.multi_dof_joint_trajectory.points.empty())
      return;

    sensor_msgs::MultiDOFJointState js;
    js.joint_names = t.multi_dof_joint_trajectory.joint_names;

    const std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>& points = t.multi_dof_joint_trajectory.points;
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint>::const_iterator
        prev = points.begin(),  // previous via point
        next = points.begin() + 1,  // currently targetted via point
        end = points.end();

    ros::Time startTime = ros::Time::now();
    while (!cancelled()) {
      ros::Duration elapsed = ros::Time::now() - startTime;
      // hop to next targetted via point
      while (next != end && elapsed > next->time_from_start) {
        ++prev;
        ++next;
      }
      if (next == end)
        break;

      double duration = (next->time_from_start - prev->time_from_start).toSec();
      ROS_DEBUG("elapsed: %.3f via points %td,%td / %td  alpha: %.3f", elapsed.toSec(), prev - points.begin(),
                next - points.begin(), end - points.begin(),
                duration > std::numeric_limits<double>::epsilon() ?
                (elapsed - prev->time_from_start).toSec() / duration :
                1.0);
      interpolate(js, *prev, *next, elapsed);
      updateTransforms(js);
      rate_.sleep();
    }
    if (cancelled())
      return;

    ros::Duration elapsed = ros::Time::now() - startTime;
    ROS_DEBUG("elapsed: %.3f via points %td,%td / %td  alpha: 1.0", elapsed.toSec(), prev - points.begin(),
              next - points.begin(), end - points.begin());

    // publish last point
    interpolate(js, *prev, *prev, prev->time_from_start);
    updateTransforms(js);

    ROS_DEBUG("Fake execution of trajectory: done");
  }
};

}  // namespace moveit_fake_controller_manager

#endif
