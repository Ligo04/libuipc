/*
 * NVIDIA contributions: state and interfaces for the contact-tabular vertex
 * cover, concurrent tree chains, stored-pair judging and skins, BVH topology
 * retention, the fused detect prefix, the in-kernel TOI minimum and read-back
 * buffers; for the certified cover EE screens, the kept-tile scatter, the
 * detection hook before the candidate-count read-back and the EE TOI launch
 * beside the other TOI launches.
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
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <array>
#include <collision_detection/filters/simplex_bvh_query.h>
#include <collision_detection/filters/simplex_query_trace.h>

namespace uipc::backend::cuda
{
class BVHSimplexPipeline
{
  public:
    using DetectInfo       = SimplexTrajectoryFilter::DetectInfo;
    using FilterActiveInfo = SimplexTrajectoryFilter::FilterActiveInfo;
    using FilterTOIInfo    = SimplexTrajectoryFilter::FilterTOIInfo;
    using PendingActiveSet = SimplexTrajectoryFilter::PendingActiveSet;
    void                                         init();
    bool                                         single_ef = false;
    SimplexBVHQuery                              simplex_query;
    std::unique_ptr<simplex_query_trace::Writer> query_trace;
    void detect(DetectInfo& info, simplex_query_trace::Site site = {});
    // Repeat the last detect with a fresh common query (UIPC_CHECK_STORED_PAIRS=1);
    // throws unless the candidate multisets match.
    void check_detect(DetectInfo& info);
    // filter_active in two phases: the launch enqueues the compaction and
    // the copy of its four counts, the finish sizes the active lists
    void launch_filter_active(FilterActiveInfo& info);
    void finish_filter_active(FilterActiveInfo& info);
    void filter_toi(FilterTOIInfo& info);

    /****************************************************
    *                   Broad Phase
    ****************************************************/

    cuda_tool::DeviceBuffer<AABB>   codim_point_aabbs;
    cuda_tool::DeviceBuffer<AABB>   point_aabbs;
    cuda_tool::DeviceBuffer<AABB>   edge_aabbs;
    cuda_tool::DeviceBuffer<AABB>   triangle_aabbs;
    cuda_tool::DeviceBuffer<IndexT> codim_point_bids;
    cuda_tool::DeviceBuffer<IndexT> point_bids;
    cuda_tool::DeviceBuffer<IndexT> edge_bids;
    cuda_tool::DeviceBuffer<IndexT> triangle_bids;
    cuda_tool::DeviceBuffer<IndexT> codim_point_cids;
    cuda_tool::DeviceBuffer<IndexT> point_cids;
    cuda_tool::DeviceBuffer<IndexT> edge_cids;
    cuda_tool::DeviceBuffer<IndexT> triangle_cids;

    using ThisBVH = InfoStacklessBVH;

    ThisBVH              lbvh_T;
    ThisBVH::QueryBuffer candidate_AllP_CodimP_pairs;
    ThisBVH::QueryBuffer candidate_CodimP_AllE_pairs;
    ThisBVH::QueryBuffer candidate_AllE_AllE_pairs;
    ThisBVH::QueryBuffer candidate_AllP_AllT_pairs;
    bool                 check_stored_pairs = false;
    bool                 direct_queries     = false;

    // FP32 inputs of the trajectory leaf tests' first rejection, per vertex
    // and detect: position and d_hat, step displacement and thickness.
    cuda_tool::DeviceBuffer<float4> f32_positions;
    cuda_tool::DeviceBuffer<float4> f32_steps;
    // The swept boxes' end points x + alpha dx in FP32, per vertex and
    // detect (the start points are f32_positions), for the prefix kernel.
    cuda_tool::DeviceBuffer<float4> f32_ends;

    // The TOI minimum's slot (toi_min.h): key, finished blocks.
    cuda_tool::DeviceBuffer<unsigned long long> toi_min_slot;
    // the EE TOI launch runs here, beside the other TOI launches
    cudaStream_t                      toi_side = nullptr;
    cudaEvent_t                       toi_fork = nullptr;
    cudaEvent_t                       toi_join = nullptr;
    cuda_tool::DeviceBuffer<IndexT>   selected_counts;
    cuda_tool::ReadbackBuffer<IndexT> selected_counts_readback;

    cuda_tool::DeviceBuffer<Vector4i> temp_PTs;
    cuda_tool::DeviceBuffer<Vector4i> temp_EEs;
    cuda_tool::DeviceBuffer<Vector3i> temp_PEs;
    cuda_tool::DeviceBuffer<Vector2i> temp_PPs;

    cuda_tool::DeviceBuffer<Vector4i> PTs;
    cuda_tool::DeviceBuffer<Vector4i> EEs;
    cuda_tool::DeviceBuffer<Vector3i> PEs;
    cuda_tool::DeviceBuffer<Vector2i> PPs;


    /****************************************************
    *                   CCD TOI
    ****************************************************/

    cuda_tool::DeviceBuffer<Float> tois;  // PP, PE, PT, EE
};

}  // namespace uipc::backend::cuda
