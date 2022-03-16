cmake_minimum_required(VERSION 3.0)
project(aut_nav)
set(CMAKE_CXX_STANDARD 17)

find_package( OpenCV REQUIRED )

include_directories(include)
include_directories(include/fm_planner)

set(CMAKE_BUILD_TYPE Release) 
   
find_package(Clothoids REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/libs/Clothoids/share/Clothoids)


include_directories(${CLOTHOIDS_INCLUDE_DIR})

add_executable(${CMAKE_PROJECT_NAME}
  source/fm_planner/asyplot.cpp
  source/fm_planner/dp_smoothing.cpp
  source/fm_planner/dubins.cpp
  source/fm_planner/planner.cpp

  source/hardwareglobalinterface.cpp
  source/main.cpp
  source/MapManager.cpp
  source/path.cpp
  source/pathpoint.cpp
  source/Publisher.cpp
  source/Replier.cpp
  source/RequesterSimple.cpp
  source/Subscriber.cpp
 # source/utils.cpp
  source/voronoi.cpp
  source/Prediction.cpp
  source/doordisinfection.cpp
  source/Navigation.cpp
  source/coverage.cpp
  source/rendezvous.cpp
  source/taskexecutor.cpp
  source/zmq_services.cpp
  source/graph.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME} pthread zmq ${OpenCV_LIBS} ${CLOTHOIDS_LIBRARIES} -fopenmp yaml-cpp stdc++fs tcmalloc)

