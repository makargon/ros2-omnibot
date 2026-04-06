cmake_minimum_required(VERSION 3.8)
project(omnibot_cv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

add_executable(image_undistort_node src/image_undistort_node.cpp)
ament_target_dependencies(image_undistort_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(image_undistort_node ${OpenCV_LIBS})

add_executable(aruco_pose_node src/aruco_pose_node.cpp)
target_include_directories(aruco_pose_node PRIVATE include)
target_sources(aruco_pose_node PRIVATE
  src/aruco_geometry.cpp
  src/world_pose_tracker.cpp
)
ament_target_dependencies(aruco_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_geometry_msgs
)
target_link_libraries(aruco_pose_node ${OpenCV_LIBS})

install(TARGETS
  image_undistort_node
  aruco_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/capture_calibration_frames.py
  scripts/calibrate_camera.py
  scripts/generate_aruco_field.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
