#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Joint.hh> 
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ControllerPlugin {
    class InvertedPendulumController : 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate,
        public gz::sim::ISystemPreUpdate {
public:
    // Configure: 一次性查找并缓存需要的实体
    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &) override
    {
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

        this->executor_->add_node(ros_node_);

        // spin ROS2 node
        ros_spin_thread_ = std::thread([this]() {
            executor_->spin();
        });

    }

    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &_ecm) override
    {
        if (this->jointEntities_.size() < this->targets_.size())
            this->CacheJointEntities(_ecm);

        this->EnsureStateComponents(_ecm);
        
        if (!this->is_initial_position_set){
            this->SetInitialPosition(_ecm);
        }
    }

    void PostUpdate(const gz::sim::UpdateInfo &,
                    const gz::sim::EntityComponentManager &_ecm) override {
        // Post-update logic here
        
        std_msgs::msg::Float64MultiArray msg;

        // 遍历打印缓存实体关节的角度和位置
        for (const auto &ent_ : this->jointEntities_) {
            // const std::string &name = ent_.first;
            const gz::sim::Entity &ent = ent_.second;
            
            // 读取q
            if(const auto *pos = _ecm.Component<gz::sim::components::JointPosition>(ent)) {
                const auto &vector_ = pos->Data();
                if (!vector_.empty()) {
                    double q = vector_[0];
                    msg.data.push_back(q);
                   // std::cout << "[Controller_Plugin] joint_name: :["<< name <<"] position: " << q ; 
                }
            }else{
                //std::cout << "[Controller_Plugin] joint_name: :["<< name <<"] position: NAN" ;
            }

            // 读取q_dot
            if (const auto *vel = _ecm.Component<gz::sim::components::JointVelocity>(ent)){
                const auto &vector_ = vel->Data();
                if (!vector_.empty()) {
                    double q_dot = vector_[0];
                    msg.data.push_back(q_dot);
                   // std::cout << " velocity: " << q_dot << std::endl;
                }
            }else{
                //std::cout << " velocity: NAN" << std::endl;
            }
        }

        this->joint_pub_->publish(msg);
    }

    ~InvertedPendulumController() {
        if (this->executor_) {
            this->executor_->cancel();
        }
        if (this->ros_spin_thread_.joinable())
            ros_spin_thread_.join();
        if (this->owns_context_ && rclcpp::ok())
            rclcpp::shutdown();
    }

private:
    void CacheJointEntities(gz::sim::EntityComponentManager &_ecm) {
        _ecm.Each<gz::sim::components::Joint, gz::sim::components::Name>(
            [&](const gz::sim::Entity &_ent,
                 const gz::sim::components::Joint *,
                 const gz::sim::components::Name *_name)->bool
            {
                if (!_name) return true;
                const std::string n = _name->Data();
                std::cout << "[ControllerPlugin] joint entity: " << n << " ent=" << _ent << '\n';
                for (const auto &t : this->targets_)
                {
                    if (n == t ||
                        (n.size() >= t.size() && n.compare(n.size() - t.size(), t.size(), t) == 0) ||
                        (n.find(t) != std::string::npos))
                    {
                        this->jointEntities_[t] = _ent;
                        std::cout << "[ControllerPlugin] cached " << t << " -> " << _ent << '\n';
                        break;
                    }
                }
                return true;
            });
    }

    void EnsureStateComponents(gz::sim::EntityComponentManager &_ecm) {
        for (const auto &ent_ : this->jointEntities_)
        {
            const auto &ent = ent_.second;

            if (!_ecm.Component<gz::sim::components::JointPosition>(ent))
            {
                _ecm.CreateComponent(ent, gz::sim::components::JointPosition());
                std::cout << "[ControllerPlugin] created JointPosition component for entity " << ent << '\n';
            }

            if (!_ecm.Component<gz::sim::components::JointVelocity>(ent))
            {
                _ecm.CreateComponent(ent, gz::sim::components::JointVelocity());
                std::cout << "[ControllerPlugin] created JointVelocity component for entity " << ent << '\n';
            }
        }
    }

    void SetInitialPosition(gz::sim::EntityComponentManager &_ecm) {
        this->EnsureStateComponents(_ecm);
        std::cout << "[Controller_Plugin] Start initial position set\n";

        for(const auto &ent_ : this->jointEntities_){
            const auto &name = ent_.first;
            const gz::sim::Entity &ent = ent_.second;
            //get component from ent
            if (name == this->targets_[0]){
                auto *reset = _ecm.Component<gz::sim::components::JointPositionReset>(ent);
                if (!reset) {
                    _ecm.CreateComponent(ent,
                        gz::sim::components::JointPositionReset({this->initial_position}));
                }else{
                    auto &data = reset -> Data();
                    if(data.empty()){
                        data.push_back(this->initial_position);
                    }else{
                        data[0] = this->initial_position;
                    }
                }
                _ecm.SetChanged(ent,
                    gz::sim::components::JointPositionReset::typeId,
                    gz::sim::ComponentState::OneTimeChange);
                std::cout << "[Controller_Plugin] Initial position setted!\n";
            }

        }

        this->is_initial_position_set = true;
    }

    const std::vector<std::string> targets_{"pivot_joint"};
    std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_spin_thread_;
    bool owns_context_{false};
    bool is_initial_position_set{false};
    double initial_position{0.5};
};
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
