cmake_minimum_required(VERSION 2.8.3)
project(mBot_Ranger_ROS_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()
add_service_files(
  FILES
  MakeBlockMover.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
)

catkin_install_python(PROGRAMS nodes/mBot_Ranger_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
