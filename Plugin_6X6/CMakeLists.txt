cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_library(Plugin_6X6 SHARED Plugin_6X6.cc)
target_link_libraries(Plugin_6X6 ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

#add_executable(P6X6 Plugin_6X6.cc)
#target_link_libraries(P6X6 ${GAZEBO_LIBRARIES})
