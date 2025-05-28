#############################################################
#                                                           #
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#                                                           #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(direct_lidar_odometry)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package( PCL REQUIRED )

find_package( Eigen3 REQUIRED )
find_package(Threads REQUIRED)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 1 4)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
add_definitions(${PCL_DEFINITIONS})
rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/save_pcd.srv
        srv/save_traj.srv
    DEPENDENCIES
        
)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(nanoflann ${PCL_LIBRARIES})

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)

# Odometry Node
add_executable(dlo_odom_node src/dlo/odom_node.cc src/dlo/odom.cc)
ament_target_dependencies(dlo_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp "pcl_ros" "nav_msgs" "tf2_ros" "sensor_msgs" "pcl_conversions" "geometry_msgs" "rclcpp")

# Mapping Node
add_executable (dlo_map_node src/dlo/map_node.cc src/dlo/map.cc)
ament_target_dependencies(dlo_map_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads "pcl_ros" "nav_msgs" "tf2_ros" "sensor_msgs" "pcl_conversions" "geometry_msgs" "rclcpp")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)

ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros)
ament_package()

add_compile_options(-std=c++14)
link_directories(${PCL_LIBRARY_DIRS})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})

target_compile_options(dlo_map_node PRIVATE ${OpenMP_FLAGS})
