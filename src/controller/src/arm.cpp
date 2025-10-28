#include <array>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>

#include <controller/controller_interface.hpp>
#include <controller/arm.hpp>


void ArmClass::ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm) {
    if (auto ctrl = this->controller_.lock()) {
        this->ArmPositionSet(_ecm,this->initial_q_);
    }
};

void ArmClass::Update() {
    if (auto ctrl = this->controller_.lock()) {
        this->now_q_ = ctrl->GetJointPosition();
        this->now_q_dot_ = ctrl->GetJointVelocity();
        std::cout << "[ControllerPlugin] Arm updated!\n";
    }
};

void ArmClass::ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q) {
    if (auto ctrl = this->controller_.lock()) {
        ctrl->SetJointPosition(_ecm, target_q);
    }
};