#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
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
#include <gz/plugin/Register.hh>

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

    // 创建Arm对象
    this->Arm_ = std::make_shared<ArmClass>(
            std::array<double, 6>{0,0,0,0,0,0},
            this->joint_names_);
    this->Arm_->SetController(this);
}

void ControllerPlugin::Controller::PreUpdate
    (const gz::sim::UpdateInfo &,
        gz::sim::EntityComponentManager &_ecm){
    if (this->jointEntities_.size() < this->joint_names_.size())
        this->CacheJointEntities(_ecm);

    this->EnsureStateComponents(_ecm);

    if (!this->Arm_->_is_initialed) {
        this->Arm_->ArmJointPositionInit(_ecm);
    }
    this->Arm_->Update(_ecm);
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
        joint_state.joint_position = 0.0;
        joint_state.joint_velocity= 0.0;
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
                jointPosition_[name] = q;
            }
        }

        // 读取q_dot
        if (const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(ent)){
            const auto &vector_ = vel->Data();
            if (!vector_.empty()) {
                double q_dot = vector_[0];
                joint_state.joint_velocity = q_dot;
                jointVelocity_[name] = q_dot;
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

GZ_ADD_PLUGIN(
    ControllerPlugin::Controller,
    gz::sim::System,
    ControllerPlugin::Controller::ISystemConfigure,
    ControllerPlugin::Controller::ISystemPostUpdate,
    ControllerPlugin::Controller::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    ControllerPlugin::Controller,
    "ControllerPlugin::Controller"
)
