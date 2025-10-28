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
#include <vector>

void ArmClass::ArmPositionCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_[i] = data[i];
            }

            std::cout << "[Arm] Recieved Joint Position Cmd ...\n";
        }
    }
};

void ArmClass::ArmForceCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_froce_[i] = data[i];
            }

            std::cout << "[Arm] Recieved Joint Force Cmd ...\n";
        }
    }
};

void ArmClass::ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm) {
    if(!this->_is_initialed) {
        this->ArmPositionSet(_ecm,this->initial_q_);
        this->_is_initialed = true;
    }
};

void ArmClass::Update(gz::sim::EntityComponentManager &_ecm) {
    this->now_q_ = this->controller_->GetJointPosition();
    this->now_q_dot_ = this->controller_->GetJointVelocity();

    if (this->target_q_ != std::array<double, 6>{0.0,0.0,0.0,0.0,0.0,0.0}) {
        this->ArmPositionSet(_ecm, this->target_q_);
    }
};

void ArmClass::ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q) {
    this->controller_->SetJointPosition(_ecm, target_q);

    std::cout << "[Controller_Plugin] Position setted!\n";
};

void ArmClass::ArmForceSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_force) {
    this->controller_->SetJointForce(_ecm, target_force);

    std::cout << "[Controller_Plugin] Position setted!\n";
};
