# Найти пакеты
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Добавить директорию include (важно!)
include_directories(include)

# Создать исполняемый файл
add_executable(encoder_node src/encoder_node.cpp)

# Подключить заголовки и библиотеки
target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(encoder_node
  ${LGPIO_LIB}
  rclcpp::rclcpp
  std_msgs::std_msgs
)