#pragma once
#include <array>
#include <gz/sim/System.hh>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <thread>
#include <rclcpp/executors.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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
            ->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/Arm/joint_position_cmd", 10, 
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
                {this->ArmPositionCmdCallback(msg);});
        this->joint_force_sub_ = this->ros_node_
            ->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/Arm/joint_force_cmd", 10, 
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
                {this->ArmForceCmdCallback(msg);});
        this->joint_velocity_sub_ = this->ros_node_
            ->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/Arm/joint_velocity_cmd", 10, 
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
                {this->ArmVelocityCmdCallback(msg);});
        this->drag_to_joint_position_sub_ = this->ros_node_
            ->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/Arm/drag_to_joint_pisition_cmd", 10, 
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
                {this->ArmDragtoJointPositionCmdCallback(msg);});
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

    // 直接调用cotroller_interface接口的底层方法
    void ArmPositionCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void ArmVelocityCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void ArmForceCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void ArmDragtoJointPositionCmdCallback(std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                        const std::array<double, 6> &target_q);

    void ArmForceSet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q_dot);

    void ArmVelocitySet(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_force);

    void ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm);

    void SetController(ControllerInterface* ctrl) { controller_ = ctrl; };

    // 执行信息获取 以及底层接口调用
    void Update(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm);

    // 进一步封装的高级接口

    /**
     * @brief 将机械臂的连杆拖动到轴坐标系指定位置，同时清空轴速度模拟静止释放
     * 
     * @param _ecm gz::sim::EntityComponentManage
     * @param target_q 目标轴坐标位置
     * @param is_velocity_rezero 是否归零速度
     */
    void DragToJointPosition(gz::sim::EntityComponentManager &_ecm,
                            const std::array<double, 6> &target_q,
                            const bool is_velocity_rezero = true);

    // ROS2 Node
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>executor_;
    std::thread ros_spin_thread_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_force_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr drag_to_joint_position_sub_;
    bool owns_context_{false};

    bool _is_initialed{false};
    bool _is_forcecmd_done{true};
    bool _is_positioncmd_done{true};
    bool _is_velocitycmd_done{true};
    bool _is_dragcmd_done{true};
    std::array<double, 6> target_q_ {};
    std::array<double, 6> target_q_dot_{};
    std::array<double, 6> target_q_froce_ {};
private:
    std::array<double, 6> initial_q_ {};
    std::array<double, 6> now_q_ {};
    std::array<double, 6> now_q_dot_ {};
    std::array<std::string, 6> joint_name_ {};

    ControllerInterface* controller_{nullptr};
};