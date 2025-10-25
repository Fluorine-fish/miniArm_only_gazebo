# 在项目根目录执行
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/controller_with_ros/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PLUGIN_PATH="$(pwd)/install/controller_with_ros/lib:${GZ_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$(pwd)/install/controller_with_ros/lib${LD_LIBRARY_PATH}"
source install/setup.bash
gz sim inverted_pendulum_world.sdf