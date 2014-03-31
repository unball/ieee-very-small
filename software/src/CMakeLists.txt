# More details at: http://wiki.ros.org/catkin/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(unball)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  message_generation
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(DIRECTORY msg FILES
  VisionMessage.msg
  StrategyMessage.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  DEPENDS 
    gazebo_ros
  CATKIN_DEPENDS
    message_runtime
  INCLUDE_DIRS 
  LIBRARIES 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)

add_executable(strategy_node
  strategy/strategy_node.cpp
  strategy/strategy.cpp
  strategy/robot.cpp)
target_link_libraries(strategy_node ${catkin_LIBRARIES})

add_executable(vision_node
  vision/vision_node.cpp
  vision/vision.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(dummy_camera dummy_camera/dummy_camera.cpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES})

add_executable(test_dummy_camera dummy_camera/test/test_dummy_camera.cpp)
target_link_libraries(test_dummy_camera ${catkin_LIBRARIES})
