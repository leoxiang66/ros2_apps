要用 ROS 2 写一个简单的 C++ "Hello World" 示例，可以按照以下步骤进行操作：

### 1. 创建一个工作空间

首先，创建一个工作空间目录：

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. 创建一个新包

在 `src` 目录下创建一个名为 `my_cpp_pkg` 的新包：

```sh
cd src
ros2 pkg create --build-type ament_cmake my_cpp_pkg
```

### 3. 编写 C++ 代码

在 `my_cpp_pkg` 目录下创建一个 `src` 目录并添加一个名为 `hello_world.cpp` 的文件：

```sh
mkdir -p ~/ros2_ws/src/my_cpp_pkg/src
```

在 `src` 目录下创建 `hello_world.cpp`：

```cpp
// ~/ros2_ws/src/my_cpp_pkg/src/hello_world.cpp
#include <rclcpp/rclcpp.hpp>

class HelloWorldNode : public rclcpp::Node {
public:
  HelloWorldNode() : Node("hello_world_node") {
    RCLCPP_INFO(this->get_logger(), "Hello, ROS 2 world!");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloWorldNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 4. 编辑 CMakeLists.txt

编辑 `my_cpp_pkg` 目录下的 `CMakeLists.txt` 文件来编译 `hello_world.cpp`：

```cmake
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
```

### 5. 构建和运行

回到工作空间的根目录并构建包：

```sh
cd ~/ros2_ws
colcon build
```

编译完成后，运行节点：

```sh
source install/setup.bash
ros2 run my_cpp_pkg hello_world
```

如果一切顺利，你应该会看到控制台输出：

```
[INFO] [hello_world_node]: Hello, ROS 2 world!
```

这样，你就完成了一个简单的 ROS 2 C++ "Hello World" 示例。