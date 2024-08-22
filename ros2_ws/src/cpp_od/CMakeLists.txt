cmake_minimum_required(VERSION 3.8)
set(Python3_EXECUTABLE /home/xiang-tao/miniconda3/envs/ros2/bin/python)
project(cpp_od)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加 Project2 目录

add_subdirectory(./yolov8 yolov8)

# 包含 Project2 的头文件

include_directories(./yolov8/include)

include_directories(./include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) # Add this line
find_package(OpenCV REQUIRED) # Add this line
set(ad_interfaces_DIR ~/include/lib/cmake/ad_interfaces)
find_package(ad_interfaces REQUIRED)


include_directories(~/include/include)
include_directories(${OpenCV_INCLUDE_DIRS})





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





add_definitions(-DTRT_10)




add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

target_link_libraries(listener ad_interfaces::perception ${OpenCV_LIBS} yolov8)

target_include_directories(listener PRIVATE
  ~/include/include
  ~/include/include/perception
)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(demo ad_interfaces::perception ${OpenCV_LIBS} yolov8)

target_include_directories(demo PRIVATE
  ~/include/include
  ~/include/include/perception
)

install(TARGETS
  listener
  demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
