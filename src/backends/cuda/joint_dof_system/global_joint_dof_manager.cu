#include <joint_dof_system/global_joint_dof_manager.h>
#include <joint_dof_system/joint_dof_reporter.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalJointDofManager);

void GlobalJointDofManager::do_build() {}

void GlobalJointDofManager::Impl::update_dof_attributes()
{
    JointDofReporter::UpdateDofAttributesInfo info;
    for(auto&& [i, R] : enumerate(reporters.view()))
    {
        R->update_dof_attributes(info);
    }
}

void GlobalJointDofManager::update_dof_attributes()
{
    m_impl.update_dof_attributes();
}

void GlobalJointDofManager::add_reporter(JointDofReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.reporters.register_sim_system(*reporter);
}
}  // namespace uipc::backend::cuda
