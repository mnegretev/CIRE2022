cmake_minimum_required(VERSION 2.8.3)
project(hsrb_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/moveit_attached_object_demo.py
  scripts/moveit_cartesian_demo.py
  scripts/moveit_constraints_demo.py
  scripts/moveit_fk_demo.py
  scripts/moveit_gripper_demo.py
  scripts/moveit_ik_demo.py
  scripts/moveit_obstacles_demo.py
  scripts/moveit_obstacles_demo_takeshi.py
  scripts/moveit_pick_and_place_demo.py
  scripts/moveit_speed_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
endif()
