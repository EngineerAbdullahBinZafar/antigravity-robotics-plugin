cmake_minimum_required(VERSION 3.10)
project(antigravity_core)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Core lifecycle library
add_library(antigravity_lifecycle_node SHARED
  src/antigravity_lifecycle_node.cpp
)

target_include_directories(antigravity_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(antigravity_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Standalone executable
add_executable(antigravity_controller_node
  src/main.cpp
)

target_link_libraries(antigravity_controller_node
  antigravity_lifecycle_node
)

ament_target_dependencies(antigravity_controller_node
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS
  antigravity_lifecycle_node
  antigravity_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
