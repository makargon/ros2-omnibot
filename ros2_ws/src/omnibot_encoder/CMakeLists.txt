cmake_minimum_required(VERSION 3.8)
project(omnibot_encoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(LGPIO_LIB lgpio REQUIRED)

include_directories(include)

add_executable(encoder_node src/encoder_node.cpp)

ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
)

target_link_libraries(encoder_node ${LGPIO_LIB})

install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()