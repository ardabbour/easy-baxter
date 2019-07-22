cmake_minimum_required(VERSION 2.8.3)
project(easy_baxter)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
