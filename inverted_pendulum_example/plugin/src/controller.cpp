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
