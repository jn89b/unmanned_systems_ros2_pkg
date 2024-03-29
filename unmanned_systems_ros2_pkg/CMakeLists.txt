cmake_minimum_required(VERSION 3.5)

#change name of package to this
project(unmanned_systems_ros2_pkg)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies this will allow CPP and python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/python_node.py
  scripts/turtlebot_simple.py
  scripts/logger.py
  scripts/ros2_hw_4_node.py
  scripts/turtlebot_pid.py
  scripts/turtlebot_wp.py
  scripts/turtlebot_spawn.py
  scripts/pn.py
  scripts/evader.py
  scripts/logger_pursuit.py
  scripts/foot_ninja_1.py
  scripts/foot_ninja_2.py
  scripts/foot_ninja_spawn.py
  scripts/dungeon_master.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

#launch
install(DIRECTORY
  launch
  models
  urdf
  worlds
  assets
  DESTINATION share/${PROJECT_NAME}
)