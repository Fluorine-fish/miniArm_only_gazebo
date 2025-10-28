#include <array>
#include <memory>
#include <string>

#include <controller/controller_interface.hpp>

class ControllerInterface;

class ArmClass {
public:
    ArmClass (const std::array<double, 6> &initial_q,
            const std::array<std::string, 6> &joint_name, 
            std::shared_ptr<ControllerInterface> controller)
    : initial_q_(initial_q), joint_name_(joint_name), controller_(controller){}; 
    
    void ArmPositionSet(gz::sim::EntityComponentManager &_ecm,
                        const std::array<double, 6> &target_q);

    void ArmJointPositionInit(gz::sim::EntityComponentManager &_ecm);

    void Update();

private:
    std::array<double, 6> initial_q_ {};
    std::array<double, 6> target_q_ {};
    std::array<double, 6> now_q_ {};
    std::array<double, 6> now_q_dot_ {};
    std::array<std::string, 6> joint_name_ {};
    bool _is_initialed{false};

    std::weak_ptr<ControllerInterface> controller_;
};