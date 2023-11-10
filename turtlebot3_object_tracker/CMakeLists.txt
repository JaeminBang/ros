cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_object_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  message_generation  # 추가: message_generation 패키지 추가
)

## Uncomment the following two lines to add dependencies from this package to other packages.
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES turtlebot3_object_tracker
#   CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
#   DEPENDS system_lib
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  HumanDetection.srv
)

## Uncomment the following two lines to add dependencies from this package to other packages.
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime  # 추가: message_runtime 추가
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

