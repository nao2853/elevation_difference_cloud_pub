cmake_minimum_required(VERSION 2.8.3)
project(elevation_difference_cloud_pub)

set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
             roscpp
             tf
             nav_msgs
             sensor_msgs
             pcl_ros
             pcl_conversions
             pcl_msgs
             laser_geometry)

find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS
        roscpp
        tf
        pcl_ros
        pcl_conversions
        pcl_msgs
        laser_geometry
  INCLUDE_DIRS include
  LIBRARIES elevation_difference_cloud_pub
)

include_directories(include/elevation_difference_cloud_pub include/elevation_difference_cloud_pub/map)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(elevation_difference_cloud_pub_map
                    src/elevation_difference_cloud_pub/map/map.c)
target_link_libraries(elevation_difference_cloud_pub_map)


add_executable(elevation_difference_cloud_pub
                       src/elevation_difference_cloud_pub_node.cpp)

target_link_libraries(elevation_difference_cloud_pub
    elevation_difference_cloud_pub_map
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    elevation_difference_cloud_pub elevation_difference_cloud_pub_map
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/elevation_difference_cloud_pub/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
