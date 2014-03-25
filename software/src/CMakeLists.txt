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
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

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

catkin_package(
  DEPENDS 
    gazebo_ros
  CATKIN_DEPENDS
  INCLUDE_DIRS 
  LIBRARIES 
)

add_executable(hello_world test/hello_world.cpp)

add_executable(strategy_node
  strategy/strategy_node.cpp
  strategy/strategy.cpp)
target_link_libraries(strategy_node ${catkin_LIBRARIES})

add_executable(dummy_camera dummy_camera/dummy_camera.cpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES})
