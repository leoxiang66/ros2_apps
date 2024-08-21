cmake_minimum_required(VERSION 3.8)
project(my_custom_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_custom_msgs REQUIRED)

# Add publisher executable
add_executable(my_custom_publisher src/my_custom_publisher.cpp)
ament_target_dependencies(my_custom_publisher rclcpp my_custom_msgs)

# Add subscriber executable
add_executable(my_custom_subscriber src/my_custom_subscriber.cpp)
ament_target_dependencies(my_custom_subscriber rclcpp my_custom_msgs)

# Add service server executable
add_executable(my_custom_server src/my_custom_server.cpp)
ament_target_dependencies(my_custom_server rclcpp my_custom_msgs)

# Add service client executable
add_executable(my_custom_client src/my_custom_client.cpp)
ament_target_dependencies(my_custom_client rclcpp my_custom_msgs)

install(TARGETS
  my_custom_publisher
  my_custom_subscriber
  my_custom_server
  my_custom_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()