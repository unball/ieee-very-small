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
  freenect_camera
  freenect_launch
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(freenect_launch REQUIRED)

find_package(freenect_camera REQUIRED)

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
  strategy/robot.cpp
  strategy/ball.cpp
  strategy/action_controller.cpp
  strategy/play_controller.cpp
  strategy/play.cpp
  strategy/play1.cpp)
add_dependencies(strategy_node unball_gencpp)
target_link_libraries(strategy_node ${catkin_LIBRARIES})

add_executable(vision_node
  vision/vision_node.cpp
  vision/vision.cpp)
add_dependencies(vision_node unball_gencpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(show_kinect_camera camera/show_kinect_camera.cpp)
add_dependencies(show_kinect_camera unball_gencpp)
target_link_libraries(show_kinect_camera ${catkin_LIBRARIES})

add_executable(save_kinect_video camera/save_kinect_video.cpp)
add_dependencies(save_kinect_video unball_gencpp)
target_link_libraries(save_kinect_video ${catkin_LIBRARIES})

add_executable(dummy_camera dummy_camera/dummy_camera.cpp)
add_dependencies(dummy_camera unball_gencpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES})

add_executable(dummy_frame dummy_camera/dummy_frame.cpp)
add_dependencies(dummy_frame unball_gencpp)
target_link_libraries(dummy_frame ${catkin_LIBRARIES})

add_executable(simulator_node simulator/simulator_node.cpp)
add_dependencies(simulator_node unball_gencpp)
target_link_libraries(simulator_node ${catkin_LIBRARIES})
