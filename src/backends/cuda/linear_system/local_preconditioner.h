/*
 * NVIDIA contributions: optional fused-PCG preconditioner interfaces and the
 * default unsupported-path fallback.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <sim_system.h>
#include <linear_system/global_linear_system.h>

namespace uipc::backend::cuda
{
class DiagLinearSubsystem;

class LocalPreconditioner : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class InitInfo
    {
      public:
    };

    class BuildInfo
    {
      public:
        void connect(DiagLinearSubsystem* system);

      private:
        friend class LocalPreconditioner;
        DiagLinearSubsystem* m_subsystem = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo& info)   = 0;
    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) = 0;
    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) = 0;
    virtual bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info)
    {
        return false;
    }

  private:
    friend class GlobalLinearSystem;

    virtual void do_build() final override;
    virtual void init();

    void assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info);
    void apply(GlobalLinearSystem::ApplyPreconditionerInfo& info);
    bool fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info);
    DiagLinearSubsystem* m_subsystem = nullptr;
};
}  // namespace uipc::backend::cuda
