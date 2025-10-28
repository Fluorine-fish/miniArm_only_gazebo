#pragma once
#include <array>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <rclcpp/executors.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <controller/controller_interface.hpp>


class ArmClass {
public:
    ArmClass (const std::array<double, 6> &initial_q,
            const std::array<std::string, 6> &joint_name)
    : initial_q_(initial_q), joint_name_(joint_name){
        std::cout << "[Controller_Plugin] Arm Node Init ...\n";

        //Prepare for ROS2
        if(!rclcpp::ok()) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
            this->owns_context_ = true;
        }

        //Init ROS2 Node
        this->ros_node_ = std::make_shared<rclcpp::Node>("Arm_node");
        this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        this->joint_position_sub_ = this->ros_node_
            ->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/Arm/joint_position_cmd", 10, 
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {this->ArmPositionCmdCallback(msg);});
        this->joint_force_sub_ = this->ros_node_
            ->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/Arm/joint_force_cmd", 10, 
                [this](std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {this->ArmForceCmdCallback(msg);});
        this->executor_->add_node(this->ros_node_);

        // spin ROS2 node
        ros_spin_thread_ = std::thread(
            [this]() {
                executor_->spin();
            }
        );
    }; 
    
    ~ArmClass () {
        // 释放资源
        if (this->executor_) {
            this->executor_->cancel();
        }
        if (this->ros_spin_thread_.joinable())
            ros_spin_thread_.join();
        if (this->owns_context_ && rclcpp::ok())
            rclcpp::shutdown(); 
    };

    void ArmPositionCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ArmForceCmdCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                        const std::array<double, 6> &target_q);

    void ArmForceSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_force);

    void ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm);

    void Update(gz::sim::EntityComponentManager &_ecm);

    void SetController(ControllerInterface* ctrl) { controller_ = ctrl; };

    // ROS2 Node
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>executor_;
    std::thread ros_spin_thread_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_force_sub_;
    bool owns_context_{false};

    bool _is_initialed{false};
    std::array<double, 6> target_q_ {};
    std::array<double, 6> target_q_froce_ {};
private:
    std::array<double, 6> initial_q_ {};
    std::array<double, 6> now_q_ {};
    std::array<double, 6> now_q_dot_ {};
    std::array<std::string, 6> joint_name_ {};

    ControllerInterface* controller_{nullptr};
};