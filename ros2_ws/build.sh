echo "Start building ..."

rosdep install -i --from-path src --rosdistro jazzy -y
colcon build

echo
echo "Sourcing ..."
source ./install/local_setup.bash

echo
echo