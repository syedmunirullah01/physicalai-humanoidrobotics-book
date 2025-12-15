cmake_minimum_required(VERSION 3.8)
project(isaac_ros_humanoid_system)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # For safety, set the host flags to the minimum architecture that supports CUDA
  # This is required to use vectorized operations in device code later
  set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(isaac_ros_visual_slam REQUIRED)
find_package(isaac_ros_nav2 REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)

# Include all chapters as subdirectories
add_subdirectory(chapter-1-isaac-sim)
add_subdirectory(chapter-2-isaac-ros)
add_subdirectory(chapter-3-nav2)
add_subdirectory(chapter-4-integration)

# Install documentation
install(
  FILES SUMMARY.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()