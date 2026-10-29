cmake_minimum_required(VERSION 3.10)
project(antigravity_rviz_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Check for Qt5 or Qt6 (ROS 2 Humble uses Qt5, Jazzy uses Qt6)
find_package(Qt6 COMPONENTS Widgets Core)
if(NOT Qt6_FOUND)
  find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
endif()

add_library(antigravity_rviz_panel SHARED
  src/antigravity_telemetry_panel.cpp
)

target_include_directories(antigravity_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(antigravity_rviz_panel
  rclcpp
  rviz_common
  rviz_default_plugins
  geometry_msgs
  std_msgs
  pluginlib
)

if(Qt6_FOUND)
  target_link_libraries(antigravity_rviz_panel Qt6::Widgets Qt6::Core)
else()
  target_link_libraries(antigravity_rviz_panel Qt5::Widgets Qt5::Core)
endif()

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS
  antigravity_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
