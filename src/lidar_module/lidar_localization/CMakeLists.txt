cmake_minimum_required(VERSION 3.5)
project(lidar_localization)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  roslib
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  ndt_omp
  fast_gicp
)

find_package(Boost COMPONENTS signals)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  QueryGlobalLocalization.srv
  SetGlobalMap.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions nodelet
  DEPENDS Boost EIGEN3 PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(lidar_global_localization
  src/bbs/bbs_localization.cpp
  src/ransac/voxelset.cpp
  src/ransac/ransac_pose_estimation.cpp
  src/engines/global_localization_bbs.cpp
)
target_link_libraries(lidar_global_localization
  ${OpenCV_LIBS}
)

add_executable(lidar_global_localization_node     src/lidar_global_localization_node.cpp)
add_dependencies(lidar_global_localization_node    lidar_global_localization    ${PROJECT_NAME}_gencpp)
target_link_libraries(lidar_global_localization_node
  lidar_global_localization
  ${catkin_LIBRARIES}
)
# add_subdirectory(src)
add_subdirectory(nodelets)
# add_subdirectory(nodes)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)