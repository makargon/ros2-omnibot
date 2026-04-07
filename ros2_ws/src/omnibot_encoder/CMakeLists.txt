cmake_minimum_required(VERSION 3.8)
project(omnibot_encoder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Найти зависимости ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)   # для Int32MultiArray

# Найти системную библиотеку lgpio
find_library(LGPIO_LIB lgpio REQUIRED)
if(NOT LGPIO_LIB)
  message(FATAL_ERROR "lgpio not found: sudo apt install liblgpio-dev")
endif()

# Добавить директорию с заголовками
include_directories(include)

# Создать исполняемый файл
add_executable(encoder_node src/encoder_node.cpp)

# ✅ Правильный способ подключения зависимостей ROS2
ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
)

# Добавить lgpio
target_link_libraries(encoder_node ${LGPIO_LIB})

# Установка
install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()