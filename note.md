## 1. 用gazebo将urdf转换为sdf格式

```bash
gz sdf -p miniArm.urdf > miniArm.sdf
```
## 2. gazebo生成的sdf文件碰撞体积丢失问题
- sw导出的urdf用stl文件描述碰撞体积，STL 是“表面网格”，没有封闭性和凸性信息。Gazebo 的 Bullet 或 DART 物理引擎要求碰撞体是凸形或封闭体，否则会出错（例如物体爆炸、穿透、抖动）。
> 直接使用urdf让gz自己解析

- 如果保留原始数据不简化，直接运行仿真会导致仿真崩溃
- 尝试dae格式问题仍旧存在

## 2. gazebo pulgin 作为控制器
> [tutorial url](https://gazebosim.org/api/sim/8/createsystemplugins.html)

 Controllers and systems that provide feedback based on the state of the world will need to implement `ISystemPostUpdate` to read the state at the end of an update frame, as well as `ISystemPreUpdate` to provide feedback at the beginning of the next frame. 实现 ISystemPostUpdate 和 ISystemPreUpdate 接口，前者读取状态，后者提供反馈

- 一个控制器插件的最小可编译实现是：
``` cpp
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

namespace ControllerPlugin {
    class InvertedPendulumController : 
        public gz::sim::System,
        public gz::sim::ISystemPostUpdate,
        public gz::sim::ISystemPreUpdate {
public:
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override {
        // Pre-update logic here
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override {
        // Post-update logic here
    }
    };
}

GZ_ADD_PLUGIN(
    ControllerPlugin::InvertedPendulumController,
    gz::sim::System,
    ControllerPlugin::InvertedPendulumController::ISystemPostUpdate,
    ControllerPlugin::InvertedPendulumController::ISystemPreUpdate
)
```

## 3. 参考```ignition::gazebo::systems```的接口用于编写plugin
> [reference url](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo_1_1systems.html)


## 4. cmake编译pulgin
- 在使用```gz_find_package(gz-plugin2 REQUIRED COMPOENTS register)```前，需要先使用
``` cmake
find_package(gz-cmake3 REQUIRED)
set(GZ_SIM_VER 8)
```
- gazebo系统插件只需要连接gz-sim即可
``` cmake
find_package(gz-cmake3 REQUIRED)
set(GZ_SIM_VER 8)

gz_find_package(gz-sim${GZ_SIM_VER} REQUIRED COMPONENTS register)

#Add sources for plugin
add_library(InvertedPendulumControllerPlugin SHARED src/controller.cpp)
target_link_libraries(InvertedPendulumControllerPlugin
  PUBLIC gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)
```

## 5. 使用-v参数进行调试
``` bash
gz sim inverted_pendulum_world.sdf -v 5 | grep ControllerPlugin -n
```

## 6. 使用foxglove监看ros2节点
- 安装foxglove_bridge
```bash
sudo apt install ros-<ros2-distro>-foxglove-bridge
```

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
- port参数指定websocket端口，foxglove studio连接该端口即可监看ros2节点

## 7. 为了防止在自定义的plugin启动，调用Config时其他服务未启动，请在PrePost中进行初始化
比如以下的方法需要在PrePost中调用
```cpp
    void SetInitialPosition(gz::sim::EntityComponentManager &_ecm) {
        this->EnsureStateComponents(_ecm);
        std::cout << "[Controller_Plugin] Start initial position set\n";

        for(const auto &ent_ : this->jointEntities_){
            const auto &name = ent_.first;
            const gz::sim::Entity &ent = ent_.second;
            //get component from ent
            if (name == this->targets_[0]){
                auto *reset = _ecm.Component<gz::sim::components::JointPositionReset>(ent);
                if (!reset) {
                    _ecm.CreateComponent(ent,
                        gz::sim::components::JointPositionReset({this->initial_position}));
                }else{
                    auto &data = reset -> Data();
                    if(data.empty()){
                        data.push_back(this->initial_position);
                    }else{
                        data[0] = this->initial_position;
                    }
                }
                _ecm.SetChanged(ent,
                    gz::sim::components::JointPositionReset::typeId,
                    gz::sim::ComponentState::OneTimeChange);
                std::cout << "[Controller_Plugin] Initial position setted!\n";
            }

        }

        this->is_initial_position_set = true;
    }
```