#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh> 
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <fstream>

namespace ControllerPlugin {
    class InvertedPendulumController : 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate,
        public gz::sim::ISystemPreUpdate {
public:
    // Configure: 一次性查找并缓存需要的实体
    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
        std::cerr << "[Controller_Plugin] Configure called\n";
        this->CacheJointEntities(_ecm);
    }
    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (this->jointEntities_.size() < this->targets_.size())
            this->CacheJointEntities(_ecm);

        this->EnsureStateComponents(_ecm);
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override {
        // Post-update logic here
        
        // 遍历打印缓存实体关节的角度和位置
        for (const auto &ent_ : this->jointEntities_) {
            const std::string &name = ent_.first;
            const gz::sim::Entity &ent = ent_.second;
            
            // 读取q
            if(const auto *pos = _ecm.Component<gz::sim::components::JointPosition>(ent)) {
                const auto &vector_ = pos->Data();
                if (!vector_.empty()) {
                    double q = vector_[0];
                    std::cout << "[Controller_Plugin] joint_name: :["<< name <<"] position: " << q ; 
                }
            }else{
                std::cout << "[Controller_Plugin] joint_name: :["<< name <<"] position: NAN" ;
            }

            // 读取q_dot
            if (const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(ent)){
                const auto &vector_ = vel->Data();
                if (!vector_.empty()) {
                    double q_dot = vector_[0];
                    std::cout << " velocity: " << q_dot << std::endl;
                }
            }else{
                std::cout << " velocity: NAN" << std::endl;
            }
        }
    }
private:
    void CacheJointEntities(gz::sim::EntityComponentManager &_ecm)
    {
        _ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
            [&](const gz::sim::Entity &_ent,
                 const gz::sim::components::Joint *,
                 const gz::sim::components::Name *_name)->bool
            {
                if (!_name) return true;
                const std::string n = _name->Data();
                std::cout << "[ControllerPlugin] joint entity: " << n << " ent=" << _ent << '\n';
                for (const auto &t : this->targets_)
                {
                    if (n == t ||
                        (n.size() >= t.size() && n.compare(n.size() - t.size(), t.size(), t) == 0) ||
                        (n.find(t) != std::string::npos))
                    {
                        this->jointEntities_[t] = _ent;
                        std::cout << "[ControllerPlugin] cached " << t << " -> " << _ent << '\n';
                        break;
                    }
                }
                return true;
            });
    }

    void EnsureStateComponents(gz::sim::EntityComponentManager &_ecm)
    {
        for (const auto &ent_ : this->jointEntities_)
        {
            const auto &ent = ent_.second;

            if (!_ecm.Component<gz::sim::components::JointPosition>(ent))
            {
                _ecm.CreateComponent(ent, gz::sim::components::JointPosition());
                std::cout << "[ControllerPlugin] created JointPosition component for entity " << ent << '\n';
            }

            if (!_ecm.Component<gz::sim::components::JointVelocity>(ent))
            {
                _ecm.CreateComponent(ent, gz::sim::components::JointVelocity());
                std::cout << "[ControllerPlugin] created JointVelocity component for entity " << ent << '\n';
            }
        }
    }

    const std::vector<std::string> targets_{"cart_slider", "pole_hinge"};
    std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
};
}

GZ_ADD_PLUGIN(
    ControllerPlugin::InvertedPendulumController,
    gz::sim::System,
    ControllerPlugin::InvertedPendulumController::ISystemConfigure,
    ControllerPlugin::InvertedPendulumController::ISystemPostUpdate,
    ControllerPlugin::InvertedPendulumController::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    ControllerPlugin::InvertedPendulumController, 
    "InvertedPendulumController"
)
