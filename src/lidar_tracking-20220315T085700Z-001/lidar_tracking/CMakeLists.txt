cmake_minimum_required(VERSION 3.0)
project(lidar_tracking)
set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )

include_directories(include)

set(CMAKE_BUILD_TYPE Release)

add_executable(${CMAKE_PROJECT_NAME}
  source/hardwareglobalinterface.cpp
  source/lidartracking.cpp
  source/main.cpp
  source/MapManager.cpp
  source/Publisher.cpp
  source/RequesterSimple.cpp
  source/Subscriber.cpp
)
target_link_libraries(${CMAKE_PROJECT_NAME} pthread zmq ${OpenCV_LIBS})
