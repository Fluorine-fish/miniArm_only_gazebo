#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <iostream>
#include <vector>

using namespace gz;
using namespace sim;

class JointPositionControlPlugin
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    model_ = Model(_entity);

    if (!model_.Valid(_ecm))
    {
      std::cerr << "[JointPositionControl] Invalid model.\n";
      return;
    }

    // 从 SDF 中获取关节列表
    if (_sdf->HasElement("joints"))
    {
      auto jointsElem = _sdf->GetElement("joints");
      while (jointsElem)
      {
        std::string jname = jointsElem->Get<std::string>();
        auto joint = model_.JointByName(_ecm, jname);
        if (joint != kNullEntity)
        {
          joints_.push_back(joint);
          desired_positions_.push_back(0.0);  // 初始化目标角度
        }
        else
        {
          std::cerr << "[JointPositionControl] Joint not found: " << jname << "\n";
        }
        jointsElem = jointsElem->GetNextElement("joints");
      }
    }
    else
    {
      std::cerr << "[JointPositionControl] No <joints> in SDF.\n";
    }
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &_ecm) override
  {
    if (info.paused) return;

    const double Kp = 10.0;  // 比例增益

    for (size_t i = 0; i < joints_.size(); ++i)
    {
      auto j = joints_[i];

      auto posComp = _ecm.Component<components::JointPosition>(j);
      if (!posComp || posComp->Data().empty()) continue;

      double pos = posComp->Data()[0];
      double target = desired_positions_[i];
      double error = target - pos;
      double torque = Kp * error;

      _ecm.SetComponentData<components::JointForceCmd>(j, std::vector<double>{torque});
    }
  }

private:
  Model model_;
  std::vector<Entity> joints_;
  std::vector<double> desired_positions_;
};

GZ_ADD_PLUGIN(JointPositionControlPlugin,
              gz::sim::System,
              JointPositionControlPlugin::ISystemConfigure,
              JointPositionControlPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(JointPositionControlPlugin, "miniarm::JointPositionControlPlugin")
