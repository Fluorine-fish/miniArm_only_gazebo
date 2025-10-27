#include <functional>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <string>

#include "controller/msg/arm_state.hpp"
#include "controller/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller/controller.hpp>

void ControllerPlugin::Controller::Configure(const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) {
    std::cout << "[Controller_Plugin] Plugin Init\n";
    this->CacheJointEntities(_ecm);

    //Prepare for ROS2
    if(!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
        this->owns_context_ = true;
    }

    //Init ROS2 Node
    this->ros_node_ = std::make_shared<rclcpp::Node>("Controller_node");
    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->joint_state_pub_ = this->ros_node_
        ->create_publisher<controller::msg::ArmState>("/controller/arm_state", 10);
    this->executor_->add_node(this->ros_node_);

    // spin ROS2 node
    ros_spin_thread_ = std::thread(
        [this]() {
            executor_->spin();
        }
    );
}

void ControllerPlugin::Controller::PreUpdate
    (const gz::sim::UpdateInfo &,
        gz::sim::EntityComponentManager &_ecm){
    if (this->jointEntities_.size() < this->joint_names_.size())
        this->CacheJointEntities(_ecm);

    this->EnsureStateComponents(_ecm);
};

void ControllerPlugin::Controller::PostUpdate
    (const gz::sim::UpdateInfo &,
        const gz::sim::EntityComponentManager &_ecm){
    controller::msg::ArmState msg;
    msg.joints.resize(6); //预分配内存
    for(int i = 0; i < 6; i++) {
        std::string joint_name;
        joint_name.append("joint");
        joint_name.append(std::to_string(i + 1));
        msg.joints[i].joint_name.data = joint_name;
    }

    // 遍历存储实体关节的角度和位置
    for (const auto &ent_ : this->jointEntities_) {
        const std::string &name = ent_.first;
        const gz::sim::Entity &ent = ent_.second;
        
        controller::msg::JointState joint_state;

        joint_state.joint_name.data = "Null";
        joint_state.joint_name.data = 0.0;
        joint_state.joint_name.data = 0.0;
        // 读取Name
        if (!name.empty()) {
            joint_state.joint_name.data = name;
        }
        // 读取q
        if(const auto *pos = _ecm.Component<gz::sim::components::JointPosition>(ent)) {
            const auto &vector_ = pos->Data();
            if (!vector_.empty()) {
                double q = vector_[0];
                joint_state.joint_position = q;
            }
        }

        // 读取q_dot
        if (const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(ent)){
            const auto &vector_ = vel->Data();
            if (!vector_.empty()) {
                double q_dot = vector_[0];
                joint_state.joint_velocity = q_dot;
            }
        }

        // 存储
        for (auto &slot : msg.joints) {
            if (slot.joint_name.data == joint_state.joint_name.data) {
                slot.joint_position = joint_state.joint_position;
                slot.joint_velocity = joint_state.joint_velocity;
                break;
            }
        }
    }

    this->joint_state_pub_->publish(msg);
};

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

GZ_ADD_PLUGIN(
    ControllerPlugin::Controller,
    gz::sim::System,
    ControllerPlugin::Controller::ISystemConfigure,
    ControllerPlugin::Controller::ISystemPostUpdate,
    ControllerPlugin::Controller::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    ControllerPlugin::Controller, 
    "ControllerPlugin"
)
