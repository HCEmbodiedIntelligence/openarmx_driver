cmake_minimum_required(VERSION 3.8)
project(openarmx_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(humanoid_driver_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(openarmx_ros2_control_driver SHARED
  src/openarmx_ros2_control_driver.cpp)
target_include_directories(openarmx_ros2_control_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(openarmx_ros2_control_driver PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(openarmx_ros2_control_driver
  humanoid_driver_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs)

pluginlib_export_plugin_description_file(
  humanoid_driver_interface plugins/openarmx_driver_plugins.xml)

install(TARGETS openarmx_ros2_control_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config plugins DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_openarmx_ros2_control_driver
    test/test_openarmx_ros2_control_driver.cpp)
  target_link_libraries(
    test_openarmx_ros2_control_driver
    openarmx_ros2_control_driver)
  ament_target_dependencies(test_openarmx_ros2_control_driver
    humanoid_driver_interface
    rclcpp
    sensor_msgs
    std_msgs)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading
    humanoid_driver_interface
    pluginlib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  humanoid_driver_interface
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs)

ament_package()
