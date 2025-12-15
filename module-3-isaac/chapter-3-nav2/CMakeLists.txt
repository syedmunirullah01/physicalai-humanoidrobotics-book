cmake_minimum_required(VERSION 3.8)
project(isaac_ros_nav2)

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
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_controller_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_world_model REQUIRED)
find_package(nav2_dwb_controller REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(isaac_ros_visual_slam REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install exercise files
install(
  DIRECTORY exercises
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation files
install(
  DIRECTORY
    01-intro.mdx
    02-global-planning.mdx
    03-footstep-planning.mdx
    04-local-planning.mdx
    05-recovery-behaviors.mdx
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()