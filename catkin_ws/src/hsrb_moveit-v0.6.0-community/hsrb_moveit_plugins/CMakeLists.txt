# This package has no tests
cmake_minimum_required(VERSION 2.8.3)
project(hsrb_moveit_plugins)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  eigen_conversions
  hsrb_analytic_ik
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
  tf
  tmc_control_msgs
  tmc_robot_kinematics_model
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    control_msgs
    eigen_conversions
    hsrb_analytic_ik
    moveit_core
    moveit_ros_planning
    pluginlib
    roscpp
    tf
    tmc_control_msgs
    tmc_robot_kinematics_model
  DEPENDS
)

# for Indigo support
if(${moveit_core_VERSION} VERSION_GREATER 0.9.0)
add_compile_options(-std=c++11)
endif()

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(hsrb_moveit_kinematics)
add_subdirectory(hsrb_moveit_controller_manager)

install(
  FILES
    hsrb_moveit_kinematics_plugin_description.xml
    hsrb_moveit_controller_manager_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
