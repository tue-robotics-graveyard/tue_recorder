cmake_minimum_required(VERSION 2.8.3)
project(tue_recorder)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  rosbag
  roscpp
  sensor_msgs
  topic_tools
  message_generation
)

add_message_files( FILES Start.msg Stop.msg)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tue_recorder
  CATKIN_DEPENDS cv_bridge message_filters rosbag roscpp sensor_msgs topic_tools message_runtime
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_library(tue_recorder_base src/RecorderBase.cpp)
add_executable(kinect_recorder src/KinectRecorder.cpp ${HEADER_FILES})

target_link_libraries(kinect_recorder 
						tue_recorder_base
						${catkin_LIBRARIES}
						)
add_dependencies(tue_recorder_base ${PROJECT_NAME}_generate_messages_cpp)
