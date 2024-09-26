echo "Start building ..."
echo "请确保python路径为正确的ros2 python"
echo "which python:"
which python

rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build

echo
echo "Sourcing ..."
source ./install/local_setup.bash

echo
echo