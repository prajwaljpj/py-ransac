cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_sac)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (pcl_sac pcl_sac.cpp)
target_link_libraries (pcl_sac ${PCL_LIBRARIES})

add_executable (visualize visualize.cpp)
target_link_libraries (visualize ${PCL_LIBRARIES})
