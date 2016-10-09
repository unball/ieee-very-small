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

add_definitions(-std=c++11)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(DIRECTORY msg FILES
  VisionMessage.msg
  SimulatorMessage.msg
  StrategyMessage.msg
  KeyboardMessage.msg
  MeasurementSystemMessage.msg)
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
  src/unball/utils/point.cpp
  src/unball/utils/math.cpp
  src/unball/utils/vector.cpp
  src/unball/strategy/strategy_node.cpp
  src/unball/strategy/strategy.cpp
  src/unball/strategy/state_estimator.cpp
  src/unball/strategy/robot.cpp
  src/unball/strategy/ball.cpp
  src/unball/strategy/goals.cpp
  src/unball/strategy/player.cpp
  src/unball/strategy/goalkeeper.cpp
  src/unball/strategy/goalkeeper_kicker.cpp
  src/unball/strategy/initial_goalkeeper.cpp
  src/unball/strategy/regular_player.cpp
  src/unball/strategy/kicker_player.cpp
  src/unball/strategy/assistent_player.cpp
  src/unball/strategy/attractive_potential_field.cpp
  src/unball/strategy/parallel_potential_field.cpp
  src/unball/strategy/perpendicular_potential_field.cpp
  src/unball/strategy/repulsive_potential_field.cpp
  src/unball/strategy/selective_potential_field.cpp
  src/unball/strategy/tangential_potential_field.cpp
  src/unball/strategy/trajectory_controller.cpp)
add_dependencies(strategy_node unball_gencpp)
target_link_libraries(strategy_node ${catkin_LIBRARIES})

add_executable(simulator_node
  src/unball/simulator/simulator_node.cpp)
add_dependencies(simulator_node unball_gencpp)
target_link_libraries(simulator_node ${catkin_LIBRARIES})

add_executable(keyboard_node
  src/unball/keyboard/keyboard_node.cpp
  src/unball/keyboard/keyboard_input.cpp)
add_dependencies(keyboard_node unball_gencpp)
target_link_libraries(keyboard_node ${catkin_LIBRARIES})

add_executable(communication_node
  src/unball/communication/communication_node.cpp)
add_dependencies(communication_node unball_gencpp)
target_link_libraries(communication_node ${catkin_LIBRARIES})

add_executable(measurement_system_node
  src/unball/measurement_system/measurement_system_node.cpp)
add_dependencies(measurement_system_node unball_gencpp)
target_link_libraries(measurement_system_node ${catkin_LIBRARIES})

catkin_add_gtest(test_point
  test/utils/test_point.cpp
  src/unball/utils/point.cpp)
target_link_libraries(test_point ${catkin_LIBRARIES})

catkin_add_gtest(test_angles
  test/utils/test_angles.cpp
  src/unball/utils/vector.cpp)
target_link_libraries(test_angles ${catkin_LIBRARIES})

catkin_add_gtest(test_invert_angles
  test/utils/test_invert_angles.cpp
  src/unball/utils/point.cpp
  src/unball/utils/math.cpp)
target_link_libraries(test_invert_angles ${catkin_LIBRARIES})
