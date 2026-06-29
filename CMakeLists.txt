cmake_minimum_required(VERSION 3.20)
project(mad_icp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(mad_icp_core REQUIRED)

# lidar_odometry_node
add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
  src/lidar_odometry.cpp)

target_include_directories(lidar_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lidar_odometry_node
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_eigen_TARGETS}
  ${pcl_conversions_TARGETS}
  mad_icp_core::mad_icp_core
  pcl_common)

target_compile_features(lidar_odometry_node PUBLIC cxx_std_17)

target_compile_options(lidar_odometry_node PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:
    -Wall
    -Wextra
    -Wpedantic
    -O3
  >)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
