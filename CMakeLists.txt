cmake_minimum_required(VERSION 3.0.2)
project(drr_ctr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/ros_sync_str_motor_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
