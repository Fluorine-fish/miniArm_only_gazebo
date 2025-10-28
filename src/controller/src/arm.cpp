#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Name.hh>

#include <iostream>
#include <string>

#include <controller/arm.hpp>

template<>
void ArmClass<>::ArmJointPositionInit();

template<>
void ArmClass<>::Update(const Q_array_t &now_q);