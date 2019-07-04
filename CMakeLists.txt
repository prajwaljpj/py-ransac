cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_sac)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

#add_executable (pcl_sac pcl_sac.cpp)
#target_link_libraries (pcl_sac ${PCL_LIBRARIES})

# add_executable (cluster_extraction cluster_extraction.cpp)
# target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

# add_executable (kmeans kmeans.cpp)
# target_link_libraries (kmeans ${PCL_LIBRARIES})

add_executable (ransac ransac.cpp)
target_link_libraries (ransac ${PCL_LIBRARIES})
