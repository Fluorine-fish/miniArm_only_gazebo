#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class PID_Class {
public:
    PID_Class(const double &kp, const double &ki, const double &kd) 
    : _kp{kp}, _kd(kd), _ki(ki){ };

    double PID_Calc(const double &target, const double &now) {
        this->_now = now;
        this->_target = target; 
        this->_err = this->_target - this->_now;

        if (!(abs(_err_sum + _err) > this->_i_sum_max) && this->_ki != 0.0) {
            this->_err_sum += this->_err;
        }

        this->_p_out = this->_kp * this->_err;
        this->_i_out = this->_ki * this->_err_sum;
        this->_d_out = this->_kd * (this->_err - this->_last_err);

        this->_out = this->_p_out + this->_d_out + this->_i_out;
        this->_last_err = this->_err;

        return this->_out;
    };

    double _target{0.0};
    double _now{0.0};
    double _err{0.0};
    double _last_err{0.0};
    double _err_sum{0.0};

    double _out{0.0};
    double _p_out{0.0};
    double _d_out{0.0};
    double _i_out{0.0};
    double _i_sum_max{3.0};

    double _kp;
    double _kd;
    double _ki;
};

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
                   gz::sim::EventManager &) override;

    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(const gz::sim::UpdateInfo &,
                    const gz::sim::EntityComponentManager &_ecm) override;

    void CreatePIDController(const double &kp, const double &ki, const double &kd);

    void subscription_callback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

    ~InvertedPendulumController() {
        // 释放资源
        if (this->PIDContorller_handel) {
            delete this->PIDContorller_handel;
            this->PIDContorller_handel = nullptr;
        }
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

    // 确保每一个joint对象准备好插件交互的component
    void EnsureStateComponents(gz::sim::EntityComponentManager &_ecm) {
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

    void ForceControl(gz::sim::EntityComponentManager &_ecm) {
        this->EnsureStateComponents(_ecm);
        
        for(const auto &ent_ : this->jointEntities_) {
            const auto &name = ent_.first;
            const auto &ent = ent_.second;
            if (name == this->targets_[0]) {
                auto *force_cmd = _ecm.Component<gz::sim::components::JointForceCmd>(ent);
                if (!force_cmd) {
                    _ecm.CreateComponent(ent, 
                    gz::sim::components::JointForceCmd({this->PIDContorller_handel->_out}));
                }else{
                    auto &data = force_cmd->Data();
                    if(data.empty()){
                        data.push_back(this->PIDContorller_handel->_out);
                    }else{
                        data[0] = this->PIDContorller_handel->_out;
                    }
                }
                _ecm.SetChanged(ent, 
                gz::sim::components::JointForceCmd::typeId,
                gz::sim::ComponentState::OneTimeChange);
            }
        }
    }

    const std::vector<std::string> targets_{"pivot_joint"};
    std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr contorller_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_params_sub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_spin_thread_;
    
    // PID控制器实例句柄
    PID_Class* PIDContorller_handel{nullptr};
    bool owns_context_{false};
    bool is_initial_position_set{false};
    bool is_controller_on{true};
    double initial_position{0.5};
    double taregt_position{-0.2};
};
}