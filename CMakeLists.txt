cmake_minimum_required(VERSION 3.16)
project(mad_icp)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mad_icp_core REQUIRED)

# lidar_odometry_node
add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
  src/lidar_odometry.cpp
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
)

target_include_directories(lidar_odometry_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(lidar_odometry_node
  mad_icp_core::mad_icp_core
)

target_compile_features(lidar_odometry_node PUBLIC cxx_std_17)

target_compile_options(lidar_odometry_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
