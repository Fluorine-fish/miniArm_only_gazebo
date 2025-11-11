#include <array>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <unordered_map>

#include "controller/msg/arm_state.hpp"
#include "controller/arm.hpp"
#include "controller/controller_interface.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace ControllerPlugin {
    class Controller :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemPostUpdate, 
        public ControllerInterface,
        public std::enable_shared_from_this<Controller> {
public:
    Controller() { };

    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override;

    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override;
    void SetJointPosition(gz::sim::EntityComponentManager &_ecm, 
                          const std::array<double, 6> &target_q) override;

    void SetJointForce(gz::sim::EntityComponentManager &_ecm,
                                const std::array<double, 6> &target_froce) override;

    void SetJointVelocity(gz::sim::EntityComponentManager &_ecm,
                               const std::array<double, 6> &target_q_dot) override;

    void CacheJointEntities(gz::sim::EntityComponentManager &_ecm) override;

    void EnsureStateComponents(gz::sim::EntityComponentManager &_ecm) override;
    
    std::array<double, 6> GetJointPosition() override;

    std::array<double, 6> GetJointVelocity() override;

    ~Controller() {
        // 释放资源
        if (this->executor_) {
            this->executor_->cancel();
        }
        if (this->ros_spin_thread_.joinable())
            ros_spin_thread_.join();
        if (this->owns_context_ && rclcpp::ok())
            rclcpp::shutdown();        
    }

    //用于寻找使用的joint实体
    const std::array<std::string, 6> joint_names_ {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
    std::unordered_map<std::string, double> jointPosition_;
    std::unordered_map<std::string, double> jointVelocity_;
    //ROS2 node
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>executor_;
    std::thread ros_spin_thread_;
    rclcpp::Publisher<controller::msg::ArmState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_velocity_pub_;

    bool owns_context_{false};
    //Arm
    std::shared_ptr<ArmClass> Arm_;
};
}
