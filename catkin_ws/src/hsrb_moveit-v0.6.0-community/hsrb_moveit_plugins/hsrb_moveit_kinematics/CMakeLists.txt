set(MOVEIT_PLUGIN_LIB_NAME hsrb_moveit_kinematics)

add_library(${MOVEIT_PLUGIN_LIB_NAME}
  src/${MOVEIT_PLUGIN_LIB_NAME}.cpp
)

target_link_libraries(${MOVEIT_PLUGIN_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${MOVEIT_PLUGIN_LIB_NAME}_test
                    test/${MOVEIT_PLUGIN_LIB_NAME}-test.test
                    test/${MOVEIT_PLUGIN_LIB_NAME}-test.cpp)
  target_link_libraries(${MOVEIT_PLUGIN_LIB_NAME}_test ${catkin_LIBRARIES} ${MOVEIT_PLUGIN_LIB_NAME})
endif()

install(TARGETS ${MOVEIT_PLUGIN_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
