cmake_minimum_required(VERSION 3.5)
project(planning_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(planning_bridge_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(navigation_client src/navigation_client.cpp)
add_executable(pose_server src/pose_server.cpp)
add_executable(ugv_move src/ugv_move.cpp)
add_executable(ugv_transporting_uav_move src/ugv_transporting_uav_move.cpp)

target_include_directories(ugv_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(ugv_transporting_uav_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navigation_client
  rclcpp
  rclcpp_action
  geometry_msgs
  planning_bridge_msgs
  nav2_msgs
)

ament_target_dependencies(pose_server
  rclcpp
  tf2_ros
  planning_bridge_msgs
)

ament_target_dependencies(ugv_move
  rclcpp
  rclcpp_action
  geometry_msgs
  plansys2_executor
  planning_bridge_msgs
)

ament_target_dependencies(ugv_transporting_uav_move
  rclcpp
  rclcpp_action
  geometry_msgs
  plansys2_executor
  planning_bridge_msgs
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  navigation_client
  pose_server
  ugv_transporting_uav_move
  ugv_move
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
