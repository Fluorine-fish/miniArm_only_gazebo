#pragma once 
#include <array>
#include <gz/sim/EntityComponentManager.hh>

// Note: Keep this interface in global namespace as used by current code.
class ControllerInterface {
public:
    virtual ~ControllerInterface(); // out-of-line definition in .cpp ensures typeinfo exported

    // Pure-virtual interface methods implemented by Controller
    virtual void SetJointPosition(gz::sim::EntityComponentManager &ecm,
                                  const std::array<double, 6> &target_q) = 0;

    virtual void SetJointForce(gz::sim::EntityComponentManager &ecm,
                               const std::array<double, 6> &target_force) = 0;

    virtual void CacheJointEntities(gz::sim::EntityComponentManager &ecm) = 0;

    virtual void EnsureStateComponents(gz::sim::EntityComponentManager &ecm) = 0;

    virtual std::array<double, 6> GetJointPosition() = 0;

    virtual std::array<double, 6> GetJointVelocity() = 0;
};
