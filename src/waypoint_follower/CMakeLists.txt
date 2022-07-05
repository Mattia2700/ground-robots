cmake_minimum_required(VERSION 3.5)
project(waypoint_follower)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# add_executable(main src/main.cpp)
# add_executable(test_lifecycle src/test_transition.cpp)

# set(EXECUTABLES main test_lifecycle)

# ament_target_dependencies(main 
#   "lifecycle_msgs"
#   "rclcpp_lifecycle"
#   "std_msgs"
# )

# ament_target_dependencies(test_lifecycle 
#   "lifecycle_msgs"
#   "rclcpp_lifecycle"
#   "std_msgs"
# )

add_library(waypoint_client SHARED
  src/waypoint_follower.cpp)


target_include_directories(waypoint_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(waypoint_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_msgs"
  "geometry_msgs"
)


rclcpp_components_register_node(waypoint_client PLUGIN "waypoint_follower_cpp::FollowWaypoints" EXECUTABLE waypoint_follower)

install(TARGETS
  waypoint_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#   target_include_directories(test_lifecycle PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include>
# )

# target_include_directories(main PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include>
# )

# install(
#   TARGETS main test_lifecycle
#   DESTINATION lib/${PROJECT_NAME}
# )

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
