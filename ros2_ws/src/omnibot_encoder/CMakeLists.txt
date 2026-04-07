cmake_minimum_required(VERSION 3.8)
project(omnibot_encoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(LGPIO_LIB lgpio REQUIRED)
if(NOT LGPIO_LIB)
  message(FATAL_ERROR "lgpio library not found. Install with: sudo apt install liblgpio-dev")
endif()

include_directories(include)

add_executable(encoder_node src/encoder_node.cpp)

target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(encoder_node
  ${LGPIO_LIB}
  rclcpp::rclcpp
  std_msgs::std_msgs
)

install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()