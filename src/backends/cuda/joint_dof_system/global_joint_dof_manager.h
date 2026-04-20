#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class JointDofReporter;
class GlobalVertexManager;
class SimEngine;

/**
 * @brief Central manager for joint DOF attribute-update coordination.
 *
 * Fans out an `update_dof_attributes()` call to every registered
 * `JointDofReporter` so persistent per-joint state (e.g. revolute
 * current_angles) is re-synced with the current body attribute layout.
 */
class GlobalJointDofManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        SimSystemSlotCollection<JointDofReporter> reporters;

        void update_dof_attributes();
    };

    // Re-sync persistent joint DOF state (e.g. revolute current_angles) with
    // the current body attribute layout. Call this synchronously after any
    // write that invalidates the derived state (e.g. state restore).
    void update_dof_attributes();

  protected:
    virtual void do_build() override;

  private:
    friend class JointDofReporter;
    void add_reporter(JointDofReporter* reporter);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
