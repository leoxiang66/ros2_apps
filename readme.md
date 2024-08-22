
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
./run.sh

echo ""
```