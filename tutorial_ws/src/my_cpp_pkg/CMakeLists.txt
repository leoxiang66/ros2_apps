# ~/ros2_ws/src/my_cpp_pkg/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(hello_world src/hello_world.cpp)

# Link libraries
ament_target_dependencies(hello_world rclcpp)

install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME})

ament_package()