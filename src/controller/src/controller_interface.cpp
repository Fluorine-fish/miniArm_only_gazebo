#include "controller/controller_interface.hpp"
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <string>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller/controller.hpp>

ControllerInterface::~ControllerInterface() = default;

void ControllerPlugin::Controller::SetJointPosition(gz::sim::EntityComponentManager &_ecm,
                                                    const std::array<double, 6> &target_q) {
    std::cout << "[Controller_Plugin] Joint Position reset\n";

    for(const auto &name : this->joint_names_){
        const auto &joint_ent = this->jointEntities_[name];
        auto *reset = _ecm.Component<gz::sim::components::JointPositionReset>(joint_ent);
        int index = name[5] - '0' - 1;
        if (!reset) {
            _ecm.CreateComponent(joint_ent, 
            gz::sim::components::JointPositionReset({target_q[index]}));
        }else{
            auto &data = reset->Data();
            if(data.empty()) {
                data.push_back(target_q[index]);
            }else{
                data[0] = target_q[index];
            }
        }
        _ecm.SetChanged(joint_ent, 
                    gz::sim::components::JointPositionReset::typeId,
                    gz::sim::ComponentState::OneTimeChange);
        std::cout << "[Controller_Plugin] Joint "<< name <<"'s Position reset!\n";
    }
}

void ControllerPlugin::Controller::SetJointForce(gz::sim::EntityComponentManager &_ecm,
                                                    const std::array<double, 6> &target_froce) {
    std::cout << "[Controller_Plugin] Joint Force reset\n";

    for(const auto &name : this->joint_names_){
        const auto &joint_ent = this->jointEntities_[name];
        auto *reset = _ecm.Component<gz::sim::components::JointPositionReset>(joint_ent);
        int index = name[5] - '0' - 1;
        if (!reset) {
            _ecm.CreateComponent(joint_ent, 
            gz::sim::components::JointPositionReset({target_froce[index]}));
        }else{
            auto &data = reset->Data();
            if(data.empty()) {
                data.push_back(target_froce[index]);
            }else{
                data[0] = target_froce[index];
            }
        }
        _ecm.SetChanged(joint_ent, 
                    gz::sim::components::JointPositionReset::typeId,
                    gz::sim::ComponentState::OneTimeChange);
        std::cout << "[Controller_Plugin] Joint "<< name <<"'s Force reset!\n";
    }
}

void ControllerPlugin::Controller::SetJointVelocity(gz::sim::EntityComponentManager &_ecm,
                               const std::array<double, 6> &q_dot) {
    std::cout << "[Controller_Plugin] Joint Velocity reset\n";

    for(const auto &name : this->joint_names_){
        const auto &joint_ent = this->jointEntities_[name];
        auto *reset = _ecm.Component<gz::sim::components::JointVelocityReset>(joint_ent);
        int index = name[5] - '0' - 1;
        if (!reset) {
            _ecm.CreateComponent(joint_ent, 
            gz::sim::components::JointVelocityReset({q_dot[index]}));
        }else{
            auto &data = reset->Data();
            if(data.empty()) {
                data.push_back(q_dot[index]);
            }else{
                data[0] = q_dot[index];
            }
        }
        _ecm.SetChanged(joint_ent, 
                    gz::sim::components::JointVelocityReset::typeId,
                    gz::sim::ComponentState::OneTimeChange);
        std::cout << "[Controller_Plugin] Joint "<< name <<"'s Velocity reset!\n";
    }
}

void ControllerPlugin::Controller::CacheJointEntities
    (gz::sim::EntityComponentManager &_ecm) {
    _ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_ent,
            const gz::sim::components::Joint *,
            const gz::sim::components::Name *_name)->bool
        {
            if (!_name) return true;
            const std::string n = _name->Data();
            for (const auto &t : this->joint_names_) {
                if (n == t ||
                    (n.size() >= t.size() && n.compare(n.size() - t.size(), t.size(), t) == 0) ||
                    (n.find(t) != std::string::npos)
                ) {
                    this->jointEntities_[t] = _ent;
                    std::cout << "[ControllerPlugin] cached " << t << " -> " << _ent << '\n';
                    break;
                }
            }
            return true;
        }
    );
};

// 确保每一个joint对象准备好插件交互的component
void ControllerPlugin::Controller::EnsureStateComponents
    (gz::sim::EntityComponentManager &_ecm){
        for (const auto &ent_ : this->jointEntities_)
        {
            const auto &ent = ent_.second;

            if (!_ecm.Component<gz::sim::components::JointPosition>(ent)) {
                _ecm.CreateComponent(ent, gz::sim::components::JointPosition());
                std::cout << "[ControllerPlugin] created JointPosition component for entity " << ent << '\n';
            }

            if (!_ecm.Component<gz::sim::components::JointVelocity>(ent)) {
                _ecm.CreateComponent(ent, gz::sim::components::JointVelocity());
                std::cout << "[ControllerPlugin] created JointVelocity component for entity " << ent << '\n';
            }

            if (!_ecm.Component<gz::sim::components::JointForce>(ent)) {
                _ecm.CreateComponent(ent, gz::sim::components::JointForce());
                std::cout << "[ControllerPlugin] created JointForce component for entity " << ent << '\n';
            }
        }
};

std::array<double, 6> ControllerPlugin::Controller::GetJointPosition() {
    std::array<double, 6> joint_position = {};
    joint_position[0] = this->jointPosition_["joint1"];
    joint_position[1] = this->jointPosition_["joint2"];
    joint_position[2] = this->jointPosition_["joint3"];
    joint_position[3] = this->jointPosition_["joint4"];
    joint_position[4] = this->jointPosition_["joint5"];
    joint_position[5] = this->jointPosition_["joint6"];

    return joint_position;
};

std::array<double, 6> ControllerPlugin::Controller::GetJointVelocity() {
    std::array<double, 6> joint_velocity = {};
    joint_velocity[0] = this->jointVelocity_["joint1"];
    joint_velocity[1] = this->jointVelocity_["joint2"];
    joint_velocity[2] = this->jointVelocity_["joint3"];
    joint_velocity[3] = this->jointVelocity_["joint4"];
    joint_velocity[4] = this->jointVelocity_["joint5"];
    joint_velocity[5] = this->jointVelocity_["joint6"];

    return joint_velocity;
};
