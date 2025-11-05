#include <array>
#include <cstddef>
#include <memory>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <chrono>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "controller/msg/arm_state.hpp"
#include "controller/msg/joint_state.hpp"

#include "alg_calc/matrix.h"
#include "alg_calc/robotics.h"
#include "alg_calc/utils.h"

using namespace std::chrono_literals;

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
        this->_d_out = - this->_kd * (this->_err - this->_last_err);

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

class AlgCalcNode : public rclcpp::Node {
public:
    AlgCalcNode()
    : rclcpp::Node("alg_calc_node")
    , joint_velocity_pid_{
        PID_Class(1,0.1,1),
        PID_Class(1,0.1,0.5),
        PID_Class(1,0.1,0),
        PID_Class(0.5,0.05,0),
        PID_Class(1.5,0.05,0),
        PID_Class(0.5,0.05,0),
    }
    , joint_position_pid_{
        PID_Class(10,0.1,1),
        PID_Class(15,0.1,0.5),
        PID_Class(5,0.1,0),
        PID_Class(0.5,0.05,0),
        PID_Class(1.5,0.05,0),
        PID_Class(0.5,0.05,0),
    }
    , centroid(RC)
    , I{
        matrixf::eye<3, 3>() * 1.0f,
        matrixf::eye<3, 3>() * 1.0f,
        matrixf::eye<3, 3>() * 1.0f,
        matrixf::eye<3, 3>() * 1.0f,
        matrixf::eye<3, 3>() * 1.0f,
        matrixf::eye<3, 3>() * 1.0f,
    }
    , links{
    robotics::Link(0, 0.09955, 0.0, -PI/2, robotics::R, 0.0,
                    -PI, PI, mess[0], centroid.col(0), I[0]),
    robotics::Link(0, 0.0,    0.14, PI,   robotics::R, -PI/2,
                    -PI/2, PI/2, mess[1], centroid.col(1), I[1]),
    robotics::Link(0, 0.0,    0.0,  -PI/2, robotics::R, -PI/2,
                    -2.7925, 0.0, mess[2], centroid.col(2), I[2]),
    robotics::Link(0, 0.153,  0.0,  PI/2,  robotics::R, 0.0,
                    -PI/2, PI/2, mess[3], centroid.col(3), I[3]),
    robotics::Link(0, 0.0,    0.0,  -PI/2, robotics::R, 0.0,
                    -2.3561, 2.3561, mess[4], centroid.col(4), I[4]),
    robotics::Link(0, 0.0875, 0.0,  0.0,   robotics::R, 0.0,
                       -PI, PI, mess[5], centroid.col(5), I[5]),
    }
    , miniArm(links)
    {
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Arm/joint_force_cmd", 10);
        arm_state_sub_ = this->create_subscription<controller::msg::ArmState>(
            "/controller/arm_state", 10,
            [this](controller::msg::ArmState::SharedPtr msg) { this->subcription_callback(msg); });
        timer_ = this->create_wall_timer(1ms, [this]() { this->timer_callback(); });
    }

    void timer_callback();
    void subcription_callback(controller::msg::ArmState::SharedPtr msg);
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    rclcpp::Subscription<controller::msg::ArmState>::SharedPtr arm_state_sub_;

    PID_Class joint_velocity_pid_[6];
    PID_Class joint_position_pid_[6];
    float joint_q_[6]{};
    Matrixf<6, 1> target_tor_{};
    // link质量矩阵
    float mess[6] = {1.32f,0.649f,0.642f, 0.493f, 0.488f, 0.009};
    // link质心位置原始数据
    float RC[18] = {-0.011f, -0.027f, -0.012f, 0.0f, 0.0f, 0.0f,
                    0.038f, 0.0f, 0.008f, -0.02f, -0.08, 0.0f,
                    0.008f, -0.069f, -0.015f, -0.005f, 0.05f, 0.012f};
    // link质心位置矩阵
    Matrixf<3, 6> centroid;
    // link惯量矩阵
    Matrixf<3, 3> I[6];
    robotics::Link links[6];
    robotics::Serial_Link<6> miniArm;
};

void AlgCalcNode::timer_callback() {
    // 计算重力补偿前馈
    this->target_tor_ = this->miniArm.rne(this->joint_q_);
    // 计算PID
    for(int i = 0; i < 6; i++) {
        this->target_tor_[i][0] += 
            this->joint_velocity_pid_[i].PID_Calc(
                this->joint_position_pid_[i].PID_Calc(0.0,this->joint_q_[i]), this->joint_q_[i]);   
    }

    // 打包msg
    std_msgs::msg::Float64MultiArray msg;
    for (int i = 0; i < 6; i++) {
        msg.data.push_back(this->target_tor_[i][0]);
    }

    this->torque_pub_->publish(msg);
}

void AlgCalcNode::subcription_callback(controller::msg::ArmState::SharedPtr msg) {
    const auto &joints = msg->joints;
    if (!joints.empty()) {
        for (size_t i = 0; i < joints.size(); i++) {
            joint_q_[i] = joints[i].joint_position;
        }
    }

    RCLCPP_INFO(this->get_logger(), "JointState Updated!");
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlgCalcNode>());
    rclcpp::shutdown();
    return 0;
};
