#include <array>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>

#include <controller/controller_interface.hpp>
#include <controller/arm.hpp>
#include <vector>

void ArmClass::ArmPositionCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_[i] = data[i];
            }

            this->_is_positioncmd_done = false;
        }
    }
};

void ArmClass::ArmForceCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_froce_[i] = data[i];
            }

            this->_is_forcecmd_done = false;
        }
    }
};

void ArmClass::ArmVelocityCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_dot_[i] = data[i];
            }

            this->_is_velocitycmd_done = false;
        }
    }
};

void ArmClass::ArmDragtoJointPositionCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    if (msg) {
        const auto &data = msg->data;
        if (!data.empty() && data.size() == 6) {
            for (int i = 0; i < 6; i++) {
                this->target_q_[i] = data[i];
            }

            this -> _is_dragcmd_done = false;
        }
    }
}

void ArmClass::ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm) {
    if(!this->_is_initialed) {
        this->ArmPositionSet(_ecm,this->initial_q_);
        this->_is_initialed = true;
    }
};

// 执行Arm的状态更新 以 及控制命令
void ArmClass::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
    this->now_q_ = this->controller_->GetJointPosition();
    this->now_q_dot_ = this->controller_->GetJointVelocity();

    if (!this->_is_positioncmd_done) {
        this->ArmPositionSet(_ecm, this->target_q_);
        this->_is_positioncmd_done = true;
    }

    if (!this->_is_dragcmd_done) {
        this->DragToJointPosition(_ecm, this->target_q_);
        this->_is_dragcmd_done = true;
    }

    // 暂停则力命令不更新
    if (!_info.paused){
        if (!this->_is_forcecmd_done) {
            this->ArmForceSet(_ecm, this->target_q_froce_);
        }

        if (!this->_is_velocitycmd_done) {
            this->ArmVelocitySet(_ecm, this->target_q_dot_);
            this->_is_velocitycmd_done = true;
        }
    }
};

void ArmClass::DragToJointPosition(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q,
                            const bool is_velocity_rezero) {
    this->ArmPositionSet(_ecm, target_q);
    if (is_velocity_rezero) this->ArmVelocitySet(
        _ecm, std::array<double, 6>{0,0,0,0,0,0});

    std::cout << "[Arm] Drag to Target Joint Position!\n";
}


void ArmClass::ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q) {
    this->controller_->SetJointPosition(_ecm, target_q);

    std::cout << "[Controller_Plugin] Position setted!\n";
};

void ArmClass::ArmForceSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_force) {
    this->controller_->SetJointForce(_ecm, target_force);

    std::cout << "[Controller_Plugin] Force setted!\n";
};

void ArmClass::ArmVelocitySet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q_dot) {
    this->controller_->SetJointVelocity(_ecm, target_q_dot);

    std::cout << "[Controller_Plugin] Velocity setted!\n";
};
