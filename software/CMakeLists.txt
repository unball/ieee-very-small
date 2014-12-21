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
  StrategyMessage.msg
  KeyboardMessage.msg)
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
  include/
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
  src/unball/geometry/point.cpp
  src/unball/strategy/strategy_node.cpp
  src/unball/strategy/strategy.cpp
  src/unball/strategy/state_estimator.cpp
  src/unball/strategy/robot.cpp
  src/unball/strategy/ball.cpp
  src/unball/strategy/action_controller.cpp
  src/unball/strategy/play_controller.cpp
  src/unball/strategy/play.cpp
  src/unball/strategy/tactic.cpp
  src/unball/strategy/no_play.cpp
  src/unball/strategy/play_stop.cpp
  src/unball/strategy/play_formation_1.cpp
  src/unball/strategy/play_formation_2.cpp)
add_dependencies(strategy_node unball_gencpp)
target_link_libraries(strategy_node ${catkin_LIBRARIES})

add_executable(vision_node
  src/unball/vision/vision_node.cpp
  src/unball/vision/vision.cpp
  src/unball/vision/gui.cpp
  src/unball/vision/preprocessor.cpp
  src/unball/vision/segmenter.cpp
  src/unball/vision/tracker.cpp
  src/unball/vision/tracked_object.cpp
  src/unball/vision/tracked_field.cpp
  src/unball/vision/tracked_robot.cpp)
add_dependencies(vision_node unball_gencpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(show_kinect_camera
  src/unball/camera/show_kinect_camera.cpp)
add_dependencies(show_kinect_camera unball_gencpp)
target_link_libraries(show_kinect_camera ${catkin_LIBRARIES})

add_executable(save_kinect_video
  src/unball/camera/save_kinect_video.cpp)
add_dependencies(save_kinect_video unball_gencpp)
target_link_libraries(save_kinect_video ${catkin_LIBRARIES})

add_executable(dummy_camera
  src/unball/dummy_camera/dummy_camera.cpp)
add_dependencies(dummy_camera unball_gencpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES})

add_executable(dummy_frame
  src/unball/dummy_camera/dummy_frame.cpp)
add_dependencies(dummy_frame unball_gencpp)
target_link_libraries(dummy_frame ${catkin_LIBRARIES})

add_executable(simulator_node
  src/unball/simulator/simulator_node.cpp)
add_dependencies(simulator_node unball_gencpp)
target_link_libraries(simulator_node ${catkin_LIBRARIES})

add_executable(keyboard_node
  src/unball/keyboard/keyboard_node.cpp)
add_dependencies(keyboard_node unball_gencpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES})

catkin_add_gtest(test_point
  test/geometry/test_point.cpp
  src/unball/geometry/point.cpp)
target_link_libraries(test_point ${catkin_LIBRARIES})