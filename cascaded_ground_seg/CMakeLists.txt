cmake_minimum_required(VERSION 2.8.3)
project(cascaded_ground_seg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp	
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   CATKIN_DEPENDS cv_bridge sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#SET(CMAKE_CXX_FLAGS "-O0 -g -Wall ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(${PROJECT_NAME}_node src/cascaded_ground_seg.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(offline_gs src/offline_gs.cpp)

target_link_libraries(offline_gs
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(offline_gs_noclass src/offline_gs_noclass.cpp)

target_link_libraries(offline_gs_noclass
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)