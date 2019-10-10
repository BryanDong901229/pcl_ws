cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(pcl_visualizer_viewports)

find_package(PCL 1.7 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(GLOG_INCLUDE_DIRECTORY /usr/local/include/glog)
set(GLOG_LIB_DIR /usr/local/lib)
set(GLOG_LIBRARIES glog glog)

set(GFLAGS_INCLUDE_DIRECTORY /usr/local/include/glog)
set(GFLAGS_LIB_DIR /usr/local/lib)
set(GFLAGS_LIBRARIES gflags gflags)

include_directories(${GLOG_INCLUDE_DIRECTORY})
link_directories(${GLOG_LIB_DIR})

include_directories(${GFLAGS_INCLUDE_DIRECTORY})
link_directories(${GFLAGS_LIB_DIR})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)

target_link_libraries(pcl_visualizer_demo
   ${PCL_LIBRARIES}
   ${GLOG_LIBRARIES}
   ${GFLAGS_LIBRARIES}
)





