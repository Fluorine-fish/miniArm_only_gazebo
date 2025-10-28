#pragma once 
#include <array>
#include <gz/sim/EntityComponentManager.hh>

class ControllerInterface {
public:
    virtual void SetJointPosition(gz::sim::EntityComponentManager &_ecm, 
                          const std::array<double, 6> &target_q);
             
    virtual void CacheJointEntities(gz::sim::EntityComponentManager &_ecm);

    virtual void EnsureStateComponents(gz::sim::EntityComponentManager &_ecm);

    virtual std::array<double, 6> GetJointPosition(gz::sim::EntityComponentManager &_ecm);

    virtual std::array<double, 6> GetJointVelocity(gz::sim::EntityComponentManager &_ecm);
};