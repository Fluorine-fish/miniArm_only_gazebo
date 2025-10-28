#include <array>
#include <string>

template<std::size_t N =6>
class ArmClass {
public:
    using Q_array_t = std::array<double, N>; // 存储关节相关数据
    using String_array_t = std::array<std::string, N>;// 存储关节命名等

    ArmClass (const Q_array_t &initial_q_,
            const String_array_t &joint_name_)
    : _initial_q(initial_q_), _joint_name(joint_name_){}; 
    
    void ArmPositionSet(const Q_array_t &target_q);

    void ArmJointPositionInit();

    void Update(const Q_array_t &now_q);

private:
    Q_array_t _initial_q {};
    Q_array_t _target_q {};
    Q_array_t _now_q {};
    Q_array_t _now_q_dot {};
    String_array_t _joint_name {};
    bool _is_initialed{false};
};