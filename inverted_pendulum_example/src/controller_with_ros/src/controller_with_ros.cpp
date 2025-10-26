#include <functional>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controller_with_ros/controller_with_ros.hpp>

void ControllerPlugin::InvertedPendulumController::Configure(const gz::sim::Entity &,
                const std::shared_ptr<const sdf::Element> &,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &) {
    std::cerr << "[Controller_Plugin] Configure called\n";
    this->CacheJointEntities(_ecm);

    // Prepare for ROS2 
    if(!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc,argv);
        this->owns_context_ = true;
    }

    // init ROS2 node 
    this->ros_node_ = std::make_shared<rclcpp::Node>("inverted_pendulum_plugin");
    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->joint_pub_ = this->ros_node_
        ->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_states",10);
    this->contorller_pub_ = this->ros_node_
        ->create_publisher<std_msgs::msg::Float64MultiArray>("/controller_info",10);
    this->pid_params_sub_ = this->ros_node_
        ->create_subscription<std_msgs::msg::Float64MultiArray>("/pid_params", 10, 
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {this->subscription_callback(msg); });

    this->executor_->add_node(ros_node_);

    // spin ROS2 node
    ros_spin_thread_ = std::thread([this]() {
        executor_->spin();
    });

    // create controller
    this->CreatePIDController(2.5, 1.5, 0);
}

void ControllerPlugin::InvertedPendulumController::PreUpdate(const gz::sim::UpdateInfo &,
                gz::sim::EntityComponentManager &_ecm) {
    if (this->jointEntities_.size() < this->targets_.size())
        this->CacheJointEntities(_ecm);

    this->EnsureStateComponents(_ecm);
    
    if (!this->is_initial_position_set){
        this->SetInitialPosition(_ecm);
    }

    if (this->is_controller_on){
        this->ForceControl(_ecm);
    }
}

void ControllerPlugin::InvertedPendulumController::PostUpdate(const gz::sim::UpdateInfo &,
                    const gz::sim::EntityComponentManager &_ecm) {
        // Post-update logic here
        
        std_msgs::msg::Float64MultiArray msg;
        std_msgs::msg::Float64MultiArray controller_info;
        double now_q = 0.0;

        // 遍历打印缓存实体关节的角度和位置
        for (const auto &ent_ : this->jointEntities_) {
            // const std::string &name = ent_.first;
            const gz::sim::Entity &ent = ent_.second;
            
            // 读取q
            if(const auto *pos = _ecm.Component<gz::sim::components::JointPosition>(ent)) {
                const auto &vector_ = pos->Data();
                if (!vector_.empty()) {
                    double q = vector_[0];
                    now_q = q;
                    msg.data.push_back(q);
                }
            }

            // 读取q_dot
            if (const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(ent)){
                const auto &vector_ = vel->Data();
                if (!vector_.empty()) {
                    double q_dot = vector_[0];
                    msg.data.push_back(q_dot);
                }
            }

            // 读取Force
            if (const auto *force = _ecm.Component<gz::sim::components::JointForce>(ent)) {
                const auto &vector_ = force->Data();
                if(!vector_.empty()) {
                    double q_force = vector_[0];
                    msg.data.push_back(q_force);
                    std::cout << "[Controller_Plugin] add q_force\n";
                }else{
                    // 接受不到数据的占位
                    msg.data.push_back(0);
                }
            }
        }
        
        if (!this->PIDContorller_handel) {
            std::cerr << "[Controller_Plugin] PID controller not initialized\n";
            return;
        }
        this->PIDContorller_handel->PID_Calc(this->taregt_position, now_q);

        controller_info.data.push_back(this->PIDContorller_handel->_now);
        controller_info.data.push_back(this->PIDContorller_handel->_err);
        controller_info.data.push_back(this->PIDContorller_handel->_out);
        controller_info.data.push_back(this->PIDContorller_handel->_err_sum);

        this->contorller_pub_->publish(controller_info);
        this->joint_pub_->publish(msg);
    }

void ControllerPlugin::InvertedPendulumController::CreatePIDController(
    const double &kp, const double &ki, const double &kd) {
    if (this->PIDContorller_handel) return;
    this->PIDContorller_handel = new PID_Class(kp,ki,kd);
}

void ControllerPlugin::InvertedPendulumController::subscription_callback(
    std_msgs::msg::Float64MultiArray::SharedPtr msg){
    std::cout << "[Controller_Plugin] Got Pid_params msg ...\n";

    if(msg){
        const auto &pid_params = msg->data;
        if (!pid_params.empty() && (pid_params.size() == 4)){
            this->PIDContorller_handel->_kp = pid_params[0];
            this->PIDContorller_handel->_ki = pid_params[1];
            this->PIDContorller_handel->_kd = pid_params[2];
            this->taregt_position = pid_params[3];

            std::cout << "[Controller_Plugin] PID Parameters set as kp = " << pid_params[0] << " ki = " << pid_params[1] << " kd = " << pid_params[2] << "\n";
        }
    }else{
        std::cerr << "[Controller_Plugin] PID Parameters wrong!\n";
    }
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
