cmake_minimum_required(VERSION 3.20)
project(message_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(state_estimator_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(message_converter_node src/message_converter_node.cpp)

target_link_libraries(message_converter_node PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  pluginlib::pluginlib
)
target_include_directories(message_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


#define plugin
add_library(message_converter_plugins SHARED src/plugins/blind_state_converter_plugin.cpp)
target_include_directories(message_converter_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${state_estimator_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS})

target_link_libraries(message_converter_plugins PUBLIC
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(message_converter message_converter_plugins.xml)


install(
  TARGETS message_converter_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  message_converter_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES
  message_converter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  message_converter_plugins
)
ament_package()
