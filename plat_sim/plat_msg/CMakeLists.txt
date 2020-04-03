cmake_minimum_required(VERSION 2.8.3)
project(plat_msg)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  PlatMsgVehicleCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros rospy std_msgs message_runtime
)
