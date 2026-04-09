cmake_minimum_required(VERSION 3.8)
project(omnibot_encoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_library(LGPIO_LIB lgpio REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  include
)

# encoder
add_executable(encoder_node src/encoder_node.cpp)

ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
)

target_link_libraries(encoder_node ${LGPIO_LIB})

# odom
add_executable(odometry_node src/odometry_node.cpp)

ament_target_dependencies(odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  encoder_node
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()