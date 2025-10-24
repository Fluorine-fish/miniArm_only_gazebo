# 在项目根目录执行
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/plugin/build/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_PLUGIN_PATH="$(pwd)/plugin/build/lib:${GZ_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$(pwd)/plugin/build/lib:${LD_LIBRARY_PATH}"
gz sim inverted_pendulum_world.sdf