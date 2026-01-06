cmake_minimum_required(VERSION 3.8)
project(ur5e_unity_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "srv/CalculateICP.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge_node
)

install(PROGRAMS
  scripts/moveit_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moveit_bridge
)

install(PROGRAMS
  scripts/icp_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME icp_server
)

# Install other directories
install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
