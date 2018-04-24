cmake_minimum_required(VERSION 2.8.3)
project(rover_ctl)

find_package(catkin REQUIRED COMPONENTS
  joy
  rospy
  message_generation
  std_msgs
)


add_message_files(
 FILES
 MotorCMD.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rover_ctl
  CATKIN_DEPENDS joy rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
