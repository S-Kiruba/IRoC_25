cmake_minimum_required(VERSION 3.8)
project(plane_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(octomap REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(plane_segmentation_node src/plane_segmentation_node.cpp)

ament_target_dependencies(plane_segmentation_node 
  rclcpp   
  nav_msgs
  octomap_msgs
  geometry_msgs
  PCL
  octomap
  sensor_msgs 
  pcl_conversions 
  pcl_ros)


target_link_libraries(plane_segmentation_node ${PCL_LIBRARIES})

install(TARGETS plane_segmentation_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
