
# Instructions to run

```shell
git clone https://github.com/leoxiang66/ros2_apps.git
cd ros2_apps/
ROOT=$(pwd)
git checkout v0.1.0-feasible
git submodule update --init --recursive
cd ros2_ws/src/cpp_od/AD-interfaces/ad-interfaces
make install
cd ${ROOT}
./build.sh

# in current terminal
./run.sh
echo "先选择 1) cpp_od ==> 2) demo"

# in another terminal
echo "再选择 2) 2) py_pubsub_imgs ==> 1) talker"
```