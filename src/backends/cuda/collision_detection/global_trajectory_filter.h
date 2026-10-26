/*
 * NVIDIA contributions: line-search trajectory-filter lifecycle and
 * active-contact count interfaces.
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
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
class TrajectoryFilter;
class GlobalContactManager;
class ContactExporterManager;

class GlobalTrajectoryFilter final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class FilterTOIInfo
    {
      public:
        Float                     alpha() const noexcept { return m_alpha; }
        cuda_tool::VarView<Float> toi() const noexcept { return m_toi; }

      private:
        friend class GlobalTrajectoryFilter;
        Float                     m_alpha = 0.0;
        cuda_tool::VarView<Float> m_toi;
    };

    class DetectInfo
    {
      public:
        Float alpha() const noexcept { return m_alpha; }
        bool  reuse_bvh_topology() const noexcept
        {
            return m_reuse_bvh_topology;
        }

      private:
        friend class GlobalTrajectoryFilter;
        Float m_alpha              = 0.0;
        bool  m_reuse_bvh_topology = false;
    };

    class FilterActiveInfo
    {
      public:
        FilterActiveInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }


      private:
        friend class GlobalTrajectoryFilter;
        Impl* m_impl;
    };

    class LabelActiveVerticesInfo
    {
      public:
        LabelActiveVerticesInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }
        cuda_tool::BufferView<IndexT> vert_is_active() const noexcept;

      private:
        friend class GlobalTrajectoryFilter;
        Impl* m_impl;
    };

    class RecordFrictionCandidatesInfo
    {
      public:
    };

    class Impl
    {
      public:
        void  init();
        Float filter_toi(Float alpha);

        SimSystemSlotCollection<TrajectoryFilter> filters;
        SimSystemSlot<GlobalContactManager>       global_contact_manager;
        bool                                      friction_enabled = false;
        bool should_discard_friction_candidates                    = false;


        cuda_tool::DeviceBuffer<Float> tois;
        vector<Float>                  h_tois;
    };

    template <std::derived_from<SimSystem> T>
    SimSystemSlot<T> find()
    {
        return m_impl.filters.find<T>();
    }

    void add_filter(TrajectoryFilter* filter);
    void require_discard_friction();

  private:
    virtual void do_build() override final;
    virtual void do_apply_recover(RecoverInfo& info) override final;

    friend class SimEngine;
    friend class ContactExporterManager;
    void detect(Float alpha,
                bool reuse_bvh_topology = false);  // called by SimEngine and ContactExporterManager
    void filter_active();  // called by SimEngine and ContactExporterManager

    Float filter_toi(Float alpha);       // only called by SimEngine
    void  record_friction_candidates();  // only called by SimEngine
    friend class GlobalContactManager;
    void label_active_vertices();      // only called by GlobalContactManager
    void clear_friction_candidates();  // called by GlobalContactManager

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
