#include <joint_dof_system/joint_dof_reporter.h>
#include <joint_dof_system/global_joint_dof_manager.h>

namespace uipc::backend::cuda
{
void JointDofReporter::do_build()
{
    auto& manager = require<GlobalJointDofManager>();

    BuildInfo info;
    do_build(info);

    manager.add_reporter(this);
}

void JointDofReporter::update_dof_attributes(UpdateDofAttributesInfo& info)
{
    do_update_dof_attributes(info);
}
}  // namespace uipc::backend::cuda
