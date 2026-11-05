/*
 * NVIDIA contributions: trajectory BVH refit, retained query ordering and
 * corresponding reuse/invalidation logic; swept boxes with the step_forward
 * rounding, conservative FP32 distance and swept-motion rejection bounds,
 * queries from a contact-tabular vertex cover, concurrent tree chains on side
 * streams, stored-pair judging with per-group skins and FP32 inputs, detection
 * skins for candidate reuse, BVH topology retention across frames, the fused
 * detect prefix, the four-list active-set compaction and its pending active
 * set, one-warp CCD launch shapes, the in-kernel TOI minimum, TOI filtering
 * on a launch stream and batched, copy-free count read-backs; whole-warp
 * judging of the stored cover EE pairs by call site with certified FP32 bound
 * and gap screens, a one-pass judge scan with a kept-tile scatter, the cover
 * remap and the detection hook before the candidate-count read-back, and the
 * EE TOI launch beside the other TOI launches.
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

#include <collision_detection/filters/info_stackless_bvh_simplex_trajectory_filter.h>
#include <collision_detection/filters/compact_active_lists.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <collision_detection/filters/toi_min.h>
#include <cuda_tool/cub.h>
#include <cub/block/block_reduce.cuh>
#include <cub/block/block_scan.cuh>
#include <cuda_tool/cuda_tool.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <utils/simplex_contact_mask_utils.h>
#include <uipc/common/zip.h>
#include <utils/primitive_d_hat.h>
#include <utils/distance/float_lower_bound.h>
#include <utils/distance/swept_lower_bound.h>
#include <utils/distance/swept_lower_bound_f32.h>
#include <utils/distance/swept_lower_bound_f32_certain.h>
#include <utils/step_displacement.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo          = false;
constexpr bool PrintKernelZeroDistance = false;

namespace
{
    using namespace bvh_simplex_detail;
    constexpr SizeT max_iter = 1000;

    constexpr Float large_enough_toi = 1.1;

    /****************************************************
    *                   Broad Phase
    ****************************************************/

    // The prefix of a detect in two launches (trajectory_prefix.h), first per
    // vertex: the FP32 inputs as f32_inputs_kernel writes them (their
    // positions are also the swept boxes' start points, pos.cast<float>() in
    // detect_k1-k4) and the boxes' end points, rounded from pos + dx * alpha as
    // there. Thread 0 also clears the escape flags (the fill before
    // contained_kernel).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_vertex_prep_kernel(TrajectoryPrefix p)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i == 0 && p.validation.invalid)
        {
            const auto& v       = p.validation;
            *v.invalid          = v.initial_invalid;
            *v.topology_changed = v.initial_topology;
            *v.metadata_changed = v.initial_metadata;
            for(auto flag : v.group_escaped)
                *flag = 0;
        }
        if(i == 0 && p.escaped)
            for(int f = 0; f < 3; ++f)
                p.escaped[f] = 0;
        if(i >= p.vertices)
            return;
        const auto&   pos   = p.positions[i];
        Vector3       pos_t = p.accepted_step_endpoints ?
                                  step_position(pos, p.alpha, p.displacements[i]) :
                                  pos + p.displacements[i] * p.alpha;
        const Vector3 d     = step_displacement(p.alpha, p.displacements[i]);
        p.f32_positions[i]  = make_float4(static_cast<float>(pos.x()),
                                         static_cast<float>(pos.y()),
                                         static_cast<float>(pos.z()),
                                         static_cast<float>(p.d_hats[i]));
        p.f32_steps[i]      = make_float4(static_cast<float>(d.x()),
                                     static_cast<float>(d.y()),
                                     static_cast<float>(d.z()),
                                     static_cast<float>(p.thicknesses[i]));
        p.f32_ends[i]       = make_float4(static_cast<float>(pos_t.x()),
                                    static_cast<float>(pos_t.y()),
                                    static_cast<float>(pos_t.z()),
                                    0.0f);
    }

    __device__ Eigen::Vector3f prefix_point(const float4* points, IndexT v)
    {
        const float4 x = points[v];
        return Eigen::Vector3f{x.x, x.y, x.z};
    }

    __device__ void prefix_invalidate(const PrefixValidation& validation, IndexT* reason)
    {
        atomicExch(validation.invalid, 1);
        atomicExch(reason, 1);
    }

    // Then per primitive, one block range per family: each thread builds its
    // box as detect_k1-k4 build it (the same FP64 expansion, the same order of
    // extends), writes it with its bid and cid, copies them to the box's slot
    // of the gathered cover lists (cover_gather_kernel) and checks the box
    // against its grown copy (contained_kernel).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_prefix_kernel(TrajectoryPrefix p)
    {
        int f     = 0;
        int block = static_cast<int>(blockIdx.x);
        for(; f < TrajectoryPrefixFamilies; ++f)
        {
            int blocks = (p.family[f].n + blockDim.x - 1) / blockDim.x;
            if(block < blocks)
                break;
            block -= blocks;
        }
        if(f == TrajectoryPrefixFamilies)
        {
            const SizeT i = SizeT(block) * blockDim.x + threadIdx.x;
            const auto& v = p.validation;
            for(int r = 0; r < v.count; ++r)
            {
                const auto& range = v.ranges[r];
                if(i < range.n)
                {
                    const IndexT value =
                        range.is_matrix ? range.matrix.flatten(static_cast<int>(i)) :
                                          range.values(i);
                    if(value != range.previous[i])
                        prefix_invalidate(v, v.metadata_changed);
                }
            }
            return;
        }
        const auto& F = p.family[f];
        int         i = block * blockDim.x + threadIdx.x;
        if(i >= F.n)
            return;

        AABB   aabb;
        Float  thickness;
        Float  d_hat_expansion;
        IndexT v0;
        if(F.vertices)
        {
            auto vI         = F.vertices[i];
            thickness       = p.thicknesses[vI];
            d_hat_expansion = point_dcd_expansion(p.d_hats[vI]);
            aabb.extend(prefix_point(p.f32_positions, vI)).extend(prefix_point(p.f32_ends, vI));
            v0 = vI;
        }
        else if(F.edges)
        {
            auto eI = F.edges[i];
            thickness = edge_thickness(p.thicknesses[eI[0]], p.thicknesses[eI[1]]);
            d_hat_expansion = edge_dcd_expansion(p.d_hats[eI[0]], p.d_hats[eI[1]]);
            aabb.extend(prefix_point(p.f32_positions, eI[0]))
                .extend(prefix_point(p.f32_positions, eI[1]))
                .extend(prefix_point(p.f32_ends, eI[0]))
                .extend(prefix_point(p.f32_ends, eI[1]));
            v0 = eI[0];
        }
        else
        {
            auto fI   = F.triangles[i];
            thickness = triangle_thickness(p.thicknesses[fI[0]],
                                           p.thicknesses[fI[1]],
                                           p.thicknesses[fI[2]]);
            d_hat_expansion =
                triangle_dcd_expansion(p.d_hats[fI[0]], p.d_hats[fI[1]], p.d_hats[fI[2]]);
            aabb.extend(prefix_point(p.f32_positions, fI[0]))
                .extend(prefix_point(p.f32_positions, fI[1]))
                .extend(prefix_point(p.f32_positions, fI[2]))
                .extend(prefix_point(p.f32_ends, fI[0]))
                .extend(prefix_point(p.f32_ends, fI[1]))
                .extend(prefix_point(p.f32_ends, fI[2]));
            v0 = fI[0];
        }

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        const IndexT bid = p.v2bs[v0];
        const IndexT cid = p.contact_ids[v0];
        F.aabbs[i]       = aabb;
        F.bids[i]        = bid;
        F.cids[i]        = cid;

        const auto& validation = p.validation;
        if(validation.invalid)
        {
            if((F.previous_vertices && F.vertices[i] != F.previous_vertices[i])
               || (F.previous_edges && F.edges[i] != F.previous_edges[i])
               || (F.previous_triangles && F.triangles[i] != F.previous_triangles[i]))
                prefix_invalidate(validation, validation.topology_changed);
            if((F.previous_bids && bid != F.previous_bids[i])
               || (F.previous_cids && cid != F.previous_cids[i]))
                prefix_invalidate(validation, validation.metadata_changed);
            if(F.cached_grown)
            {
                const auto& fat   = F.cached_grown[i];
                const int   group = F.cached_groups[i];
                const bool inside = (fat.min().array() <= aabb.min().array()).all()
                                    && (aabb.max().array() <= fat.max().array()).all();
                if(!inside)
                {
                    atomicExch(validation.invalid, 1);
                    if(group == 0)
                        atomicExch(validation.group_escaped[0], 1);
                }
                if(group != 0)
                {
                    bool lower = true;
                    for(int axis = 0; axis < 3; ++axis)
                        lower &= Float(fat.min()[axis]) + F.cached_shrink[i]
                                     <= Float(aabb.min()[axis])
                                 && Float(aabb.max()[axis])
                                        <= Float(fat.max()[axis]) - F.cached_shrink[i];
                    if(!lower)
                        atomicExch(validation.group_escaped[group], 1);
                }
            }
        }

        if(F.slots)
        {
            IndexT s = F.slots[i];
            if(s >= 0)
            {
                F.gathered_aabbs[s] = aabb;
                F.gathered_bids[s]  = bid;
                F.gathered_cids[s]  = cid;
            }
        }

        if(!F.grown)
            return;
        const AABB& g      = F.grown[i];
        bool        inside = (g.min().array() <= aabb.min().array()).all()
                      && (aabb.max().array() <= g.max().array()).all();
        if(!inside)
            p.escaped[0] = 1;
        if(!F.is_query)
            return;
        const bool  cover = F.is_query[i] != 0;
        const float s     = cover ? p.cover_shrink : p.other_shrink;
        bool        lower = ((g.min().array() + s) <= aabb.min().array()).all()
                     && (aabb.max().array() <= (g.max().array() - s)).all();
        if(!lower)
            p.escaped[cover ? 1 : 2] = 1;
    }

    /****************************************************
    *                   Filter Active
    ****************************************************/

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);

        Vector2i indices = PCodimP_pairs(i);

        IndexT P0 = surf_vertices(indices(0));
        IndexT P1 = codim_vertices(indices(1));

        const auto& V0 = positions(P0);
        const auto& V1 = positions(P1);

        Float thickness = PP_thickness(thicknesses(P0), thicknesses(P1));
        Float d_hat     = PP_d_hat(d_hats(P0), d_hats(P1));

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_point_distance2(V0, V1, D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PP][low-dist] i=%d P=(%d,%d) D=%e range=(%e,%e) "
                    "thickness=%e d_hat=%e\n",
                    i,
                    P0,
                    P1,
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "P=(%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           P0,
                           P1,
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        PP = {P0, P1};
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_veritces,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);

        Vector2i indices = CodimP_AllE_pairs(i);
        IndexT   V       = codim_veritces(indices(0));
        Vector2i E       = surf_edges(indices(1));

        Vector3i vIs = {V, E(0), E(1)};
        Vector3 Ps[] = {positions(vIs(0)), positions(vIs(1)), positions(vIs(2))};

        Float thickness =
            PE_thickness(thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));

        Float d_hat = PE_d_hat(d_hats(V), d_hats(E(0)), d_hats(E(1)));

        Vector3i flag = distance::point_edge_distance_flag(Ps[0], Ps[1], Ps[2]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_edge_distance2(flag, Ps[0], Ps[1], Ps[2], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PE][low-dist] i=%d V-E=(%d,%d,%d) flag=(%d,%d,%d) "
                    "D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    flag(0),
                    flag(1),
                    flag(2),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "V-E=(%d,%d,%d), flag=(%d,%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           vIs(0),
                           vIs(1),
                           vIs(2),
                           flag(0),
                           flag(1),
                           flag(2),
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        Vector3i offsets;
        auto     dim = distance::degenerate_point_edge(flag, offsets);

        switch(dim)
        {
            case 2: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3: {
                PE = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    // The active-set filter of PT pair i.
    __device__ __forceinline__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT(
        int                              i,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::BufferView<Vector4i>  temp_PTs,
        cuda_tool::CBufferView<Float>    d_hats)
    {
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);
        auto& PT = temp_PTs(i);
        PT.setConstant(-1);

        Vector2i indices = PT_pairs(i);
        IndexT   V       = surf_vertices(indices(0));
        Vector3i F       = surf_triangles(indices(1));

        Vector4i vIs  = {V, F(0), F(1), F(2)};
        Vector3  Ps[] = {
            positions(vIs(0)), positions(vIs(1)), positions(vIs(2)), positions(vIs(3))};

        Float thickness = PT_thickness(
            thicknesses(V), thicknesses(F(0)), thicknesses(F(1)), thicknesses(F(2)));

        Float d_hat = PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

        Vector4i flag =
            distance::point_triangle_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_triangle_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PT][low-dist] i=%d V-F=(%d,%d,%d,%d) "
                    "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    vIs(3),
                    flag(0),
                    flag(1),
                    flag(2),
                    flag(3),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(
            D > 0.0, "D=%f, V F = (%d,%d,%d,%d)", D, vIs(0), vIs(1), vIs(2), vIs(3));

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "V-F=(%d,%d,%d,%d), flag=(%d,%d,%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           vIs(0),
                           vIs(1),
                           vIs(2),
                           vIs(3),
                           flag(0),
                           flag(1),
                           flag(2),
                           flag(3),
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        Vector4i offsets;
        auto     dim = distance::degenerate_point_triangle(flag, offsets);

        switch(dim)
        {
            case 2: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                IndexT V2 = vIs(offsets(2));
                PE        = {V0, V1, V2};
            }
            break;
            case 4: {
                PT = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    // The active-set filter of EE pair i.
    __device__ __forceinline__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE(
        int                              i,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  rest_positions,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::BufferView<Vector4i>  temp_EEs,
        cuda_tool::CBufferView<Float>    d_hats)
    {
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);
        auto& EE = temp_EEs(i);
        EE.setConstant(-1);

        Vector2i indices = EE_pairs(i);
        Vector2i E0      = surf_edges(indices(0));
        Vector2i E1      = surf_edges(indices(1));

        Vector4i vIs  = {E0(0), E0(1), E1(0), E1(1)};
        Vector3  Ps[] = {
            positions(vIs(0)), positions(vIs(1)), positions(vIs(2)), positions(vIs(3))};

        Float thickness = EE_thickness(thicknesses(E0(0)),
                                       thicknesses(E0(1)),
                                       thicknesses(E1(0)),
                                       thicknesses(E1(1)));

        Float d_hat =
            EE_d_hat(d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));

        Vector2 range = D_range(thickness, d_hat);

        Vector4i flag = distance::edge_edge_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Float D;
        distance::edge_edge_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][EE][low-dist] i=%d E-E=(%d,%d,%d,%d) "
                    "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    vIs(3),
                    flag(0),
                    flag(1),
                    flag(2),
                    flag(3),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }
        if(D <= range.x())
        {
            EE = vIs;
            return;
        }
        if(!is_active_D(range, D))
            return;

        Float eps_x;
        distance::edge_edge_mollifier_threshold(rest_positions(vIs(0)),
                                                rest_positions(vIs(1)),
                                                rest_positions(vIs(2)),
                                                rest_positions(vIs(3)),
                                                static_cast<Float>(1e-3),
                                                eps_x);

        if(distance::need_mollify(Ps[0], Ps[1], Ps[2], Ps[3], eps_x))
        {
            EE = vIs;
            return;
        }
        else
        {
            Vector4i offsets;
            auto     dim = distance::degenerate_edge_edge(flag, offsets);

            switch(dim)
            {
                case 2: {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    PP        = {V0, V1};
                }
                break;
                case 3: {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    IndexT V2 = vIs(offsets(2));
                    PE        = {V0, V1, V2};
                }
                break;
                case 4: {
                    EE = vIs;
                }
                break;
                default: {
                    UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
                }
                break;
            }
        }
    }

    // The PT and EE lists' active-set filter in one launch: blocks
    // [0, PT_blocks) take the PT pairs, the other blocks the EE pairs.
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k34_kernel(
        cuda_tool::CBufferView<Vector3>  PT_positions,
        cuda_tool::CBufferView<Vector2i> PT_PT_pairs,
        cuda_tool::CBufferView<IndexT>   PT_surf_vertices,
        cuda_tool::CBufferView<Vector3i> PT_surf_triangles,
        cuda_tool::CBufferView<Float>    PT_thicknesses,
        cuda_tool::BufferView<Vector2i>  PT_temp_PPs,
        cuda_tool::BufferView<Vector3i>  PT_temp_PEs,
        cuda_tool::BufferView<Vector4i>  PT_temp_PTs,
        cuda_tool::CBufferView<Float>    PT_d_hats,
        int                              PT_n,
        int                              PT_blocks,
        cuda_tool::CBufferView<Vector3>  EE_positions,
        cuda_tool::CBufferView<Vector3>  EE_rest_positions,
        cuda_tool::CBufferView<Vector2i> EE_EE_pairs,
        cuda_tool::CBufferView<Vector2i> EE_surf_edges,
        cuda_tool::CBufferView<Float>    EE_thicknesses,
        cuda_tool::BufferView<Vector2i>  EE_temp_PPs,
        cuda_tool::BufferView<Vector3i>  EE_temp_PEs,
        cuda_tool::BufferView<Vector4i>  EE_temp_EEs,
        cuda_tool::CBufferView<Float>    EE_d_hats,
        int                              EE_n)
    {
        if(static_cast<int>(blockIdx.x) < PT_blocks)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if(i < PT_n)
                InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT(
                    i,
                    PT_positions,
                    PT_PT_pairs,
                    PT_surf_vertices,
                    PT_surf_triangles,
                    PT_thicknesses,
                    PT_temp_PPs,
                    PT_temp_PEs,
                    PT_temp_PTs,
                    PT_d_hats);
        }
        else
        {
            int i = (blockIdx.x - PT_blocks) * blockDim.x + threadIdx.x;
            if(i < EE_n)
                InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE(
                    i,
                    EE_positions,
                    EE_rest_positions,
                    EE_EE_pairs,
                    EE_surf_edges,
                    EE_thicknesses,
                    EE_temp_PPs,
                    EE_temp_PEs,
                    EE_temp_EEs,
                    EE_d_hats);
        }
    }

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PP_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector2i& PP) const
        {
            return PP(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PE_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector3i& PE) const
        {
            return PE(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector4i& PT) const
        {
            return PT(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector4i& EE) const
        {
            return EE(0) != -1;
        }
    };

    // compact_active_lists(): one block per tile of one list. A block counts
    // the kept elements of its list's earlier tiles itself (the work grows
    // with the square of a list's tile count, hence CompactActiveListMaxSize),
    // then scans its tile and writes its kept elements in order.
    constexpr int CompactActiveThreads = 512;

    struct InfoStacklessBVHSimplexTrajectoryFilter_compact_active_lists
    {
        CompactActiveList list[4];
        int               first_block[4];
    };

    __global__ void __launch_bounds__(CompactActiveThreads)
        InfoStacklessBVHSimplexTrajectoryFilter_compact_active_kernel(
            InfoStacklessBVHSimplexTrajectoryFilter_compact_active_lists lists)
    {
        using BlockReduce = cub::BlockReduce<int, CompactActiveThreads>;
        using BlockScan   = cub::BlockScan<int, CompactActiveThreads>;
        __shared__ union
        {
            typename BlockReduce::TempStorage reduce;
            typename BlockScan::TempStorage   scan;
        } storage;
        __shared__ int kept_before;

        const int block = static_cast<int>(blockIdx.x);
        int       l     = 3;
        while(l > 0 && block < lists.first_block[l])
            --l;
        const CompactActiveList& L = lists.list[l];
        const int begin = (block - lists.first_block[l]) * CompactActiveThreads;

        int before = 0;
#pragma unroll 8
        for(int i = threadIdx.x; i < begin; i += CompactActiveThreads)
            before += L.in[i * L.ints] != -1 ? 1 : 0;
        before = BlockReduce(storage.reduce).Sum(before);
        if(threadIdx.x == 0)
            kept_before = before;
        __syncthreads();
        before = kept_before;

        const int i    = begin + threadIdx.x;
        const int keep = i < L.n && L.in[i * L.ints] != -1 ? 1 : 0;
        int       offset, kept;
        BlockScan(storage.scan).ExclusiveSum(keep, offset, kept);
        if(keep)
        {
            const IndexT* src = L.in + i * L.ints;
            IndexT*       dst = L.out + (before + offset) * L.ints;
            for(int k = 0; k < L.ints; ++k)
                dst[k] = src[k];
        }
        if(threadIdx.x == 0 && begin + CompactActiveThreads >= L.n)
        {
            *L.count = before + kept;
            if(L.count_copy)
                *L.count_copy = before + kept;
        }
    }

    /****************************************************
    *                   Filter TOI
    ****************************************************/

    // The TOI of PP pair i (written to PP_tois(i)).
    __device__ __forceinline__ Float InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PP(
        int                              i,
        cuda_tool::BufferView<Float>     PP_tois,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha)
    {
        auto   indices = PCodimP_pairs(i);
        IndexT V0      = surf_vertices(indices(0));
        IndexT V1      = codim_vertices(indices(1));

        Float thickness = PP_thickness(thicknesses(V0), thicknesses(V1));
        Float d_hat     = PP_d_hat(d_hats(V0), d_hats(V1));

        Vector3 VP0  = positions(V0);
        Vector3 VP1  = positions(V1);
        Vector3 dVP0 = alpha * dxs(V0);
        Vector3 dVP1 = alpha * dxs(V1);

        Float toi = large_enough_toi;

        bool faraway =
            !distance::point_point_ccd_broadphase(VP0, VP1, dVP0, dVP1, d_hat + thickness);

        if(faraway)
        {
            PP_tois(i) = toi;
            return toi;
        }

        bool hit =
            distance::point_point_ccd(VP0, VP1, dVP0, dVP1, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PP_tois(i) = toi;
        return toi;
    }

    // The PP TOIs and their minimum (toi_min.h; out: the finishing launch).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel(
        cuda_tool::BufferView<Float>     PP_tois,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n,
        TOIMinSlot                       slot,
        Float*                           out)
    {
        int                i   = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned long long key = TOIMinIdentity;
        if(i < n)
            key = toi_min_key(InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PP(
                i, PP_tois, PCodimP_pairs, codim_vertices, surf_vertices, thicknesses, positions, dxs, d_hats, eta, alpha));
        toi_min_block(key, slot, out);
    }

    // The TOI of PE pair i (written to PE_tois(i)).
    __device__ __forceinline__ Float InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PE(
        int                              i,
        cuda_tool::BufferView<Float>     PE_tois,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha)
    {
        auto     indices = CodimP_AllE_pairs(i);
        IndexT   V       = codim_vertices(indices(0));
        Vector2i E       = surf_edges(indices(1));

        Float thickness =
            PE_thickness(thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));
        Float d_hat = PE_d_hat(d_hats(V), d_hats(E(0)), d_hats(E(1)));

        Vector3 VP  = Ps(V);
        Vector3 dVP = alpha * dxs(V);

        Vector3 EP0  = Ps(E[0]);
        Vector3 EP1  = Ps(E[1]);
        Vector3 dEP0 = alpha * dxs(E[0]);
        Vector3 dEP1 = alpha * dxs(E[1]);

        Float toi = large_enough_toi;

        bool faraway = !distance::point_edge_ccd_broadphase(
            VP, EP0, EP1, dVP, dEP0, dEP1, d_hat + thickness);

        if(faraway)
        {
            PE_tois(i) = toi;
            return toi;
        }

        bool hit = distance::point_edge_ccd(
            VP, EP0, EP1, dVP, dEP0, dEP1, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PE_tois(i) = toi;
        return toi;
    }

    // The PE TOIs and their minimum (toi_min.h; out: the finishing launch).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel(
        cuda_tool::BufferView<Float>     PE_tois,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n,
        TOIMinSlot                       slot,
        Float*                           out)
    {
        int                i   = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned long long key = TOIMinIdentity;
        if(i < n)
            key = toi_min_key(InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PE(
                i, PE_tois, CodimP_AllE_pairs, codim_vertices, thicknesses, surf_edges, Ps, dxs, d_hats, eta, alpha));
        toi_min_block(key, slot, out);
    }

    // The TOI of PT pair i (written to PT_tois(i)).
    __device__ __forceinline__ Float InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PT(
        int                              i,
        cuda_tool::BufferView<Float>     PT_tois,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha)
    {
        auto     indices = PT_pairs(i);
        IndexT   V       = surf_vertices(indices(0));
        Vector3i F       = surf_triangles(indices(1));

        Float thickness = PT_thickness(
            thicknesses(V), thicknesses(F(0)), thicknesses(F(1)), thicknesses(F(2)));
        Float d_hat = PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

        Vector3 VP  = Ps(V);
        Vector3 dVP = alpha * dxs(V);

        Vector3 FP0 = Ps(F[0]);
        Vector3 FP1 = Ps(F[1]);
        Vector3 FP2 = Ps(F[2]);

        Vector3 dFP0 = alpha * dxs(F[0]);
        Vector3 dFP1 = alpha * dxs(F[1]);
        Vector3 dFP2 = alpha * dxs(F[2]);

        Float toi = large_enough_toi;

        bool faraway = !distance::point_triangle_ccd_broadphase(
            VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, d_hat + thickness);

        if(faraway)
        {
            PT_tois(i) = toi;
            return toi;
        }

        bool hit = distance::point_triangle_ccd(
            VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PT_tois(i) = toi;
        return toi;
    }

    // The PT TOIs and their minimum (toi_min.h; out: the finishing launch).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel(
        cuda_tool::BufferView<Float>     PT_tois,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n,
        TOIMinSlot                       slot,
        Float*                           out)
    {
        int                i   = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned long long key = TOIMinIdentity;
        if(i < n)
            key = toi_min_key(InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_PT(
                i, PT_tois, PT_pairs, surf_vertices, surf_triangles, thicknesses, Ps, dxs, d_hats, eta, alpha));
        toi_min_block(key, slot, out);
    }

    // The TOI of EE pair i (written to EE_tois(i)).
    __device__ __forceinline__ Float InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_EE(
        int                              i,
        cuda_tool::BufferView<Float>     EE_tois,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha)
    {
        auto     indices = EE_pairs(i);
        Vector2i E0      = surf_edges(indices(0));
        Vector2i E1      = surf_edges(indices(1));

        Float thickness = EE_thickness(thicknesses(E0(0)),
                                       thicknesses(E0(1)),
                                       thicknesses(E1(0)),
                                       thicknesses(E1(1)));

        Float d_hat =
            EE_d_hat(d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));

        Vector3 EP0  = Ps(E0[0]);
        Vector3 EP1  = Ps(E0[1]);
        Vector3 dEP0 = alpha * dxs(E0[0]);
        Vector3 dEP1 = alpha * dxs(E0[1]);

        Vector3 EP2  = Ps(E1[0]);
        Vector3 EP3  = Ps(E1[1]);
        Vector3 dEP2 = alpha * dxs(E1[0]);
        Vector3 dEP3 = alpha * dxs(E1[1]);

        Float toi = large_enough_toi;

        bool faraway = !distance::edge_edge_ccd_broadphase(
            EP0, EP1, EP2, EP3, dEP0, dEP1, dEP2, dEP3, d_hat + thickness);

        if(faraway)
        {
            EE_tois(i) = toi;
            return toi;
        }

        bool hit = distance::edge_edge_ccd(
            EP0, EP1, EP2, EP3, dEP0, dEP1, dEP2, dEP3, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        EE_tois(i) = toi;
        return toi;
    }

    // The EE TOIs and their minimum (toi_min.h; out: the finishing launch).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel(
        cuda_tool::BufferView<Float>     EE_tois,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n,
        TOIMinSlot                       slot,
        Float*                           out)
    {
        int                i   = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned long long key = TOIMinIdentity;
        if(i < n)
            key = toi_min_key(InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_EE(
                i, EE_tois, EE_pairs, surf_edges, thicknesses, Ps, dxs, d_hats, eta, alpha));
        toi_min_block(key, slot, out);
    }

}  // namespace

REGISTER_SIM_SYSTEM(InfoStacklessBVHSimplexTrajectoryFilter);

bool compact_active_lists(const std::array<CompactActiveList, 4>& lists, cudaStream_t stream)
{
    InfoStacklessBVHSimplexTrajectoryFilter_compact_active_lists args;
    int                                                          blocks = 0;
    for(int l = 0; l < 4; ++l)
    {
        if(lists[l].n > CompactActiveListMaxSize)
            return false;
        args.list[l]        = lists[l];
        args.first_block[l] = blocks;
        // an empty list still takes a block, which writes its zero count
        blocks += std::max(1, (lists[l].n + CompactActiveThreads - 1) / CompactActiveThreads);
    }
    InfoStacklessBVHSimplexTrajectoryFilter_compact_active_kernel<<<blocks, CompactActiveThreads, 0, stream>>>(
        args);
    return true;
}

void launch_trajectory_prefix(const TrajectoryPrefix& prefix, cudaStream_t stream)
{
    {
        auto k  = InfoStacklessBVHSimplexTrajectoryFilter_vertex_prep_kernel;
        int  bd = cuda_tool::best_block_dim(k);
        int  n  = prefix.vertices;
        if(n > 0 || prefix.escaped || prefix.validation.invalid)
            k<<<std::max(1, (n + bd - 1) / bd), bd, 0, stream>>>(prefix);
    }
    auto k      = InfoStacklessBVHSimplexTrajectoryFilter_prefix_kernel;
    int  bd     = cuda_tool::best_block_dim(k);
    int  blocks = 0;
    for(const auto& F : prefix.family)
        blocks += (F.n + bd - 1) / bd;
    blocks += (prefix.validation.work + bd - 1) / bd;
    if(blocks > 0)
        k<<<blocks, bd, 0, stream>>>(prefix);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_build(BuildInfo&)
{
    auto& config = world().scene().config();
    auto  method = config.find<std::string>("collision_detection/method");
    if(method->view()[0] != "info_stackless_bvh")
    {
        throw SimSystemException("Info stackless BVH unused");
    }

    m_impl.init();
}

void BVHSimplexPipeline::init()
{
    simplex_query.method = single_ef ? SimplexBVHQuery::Method::SingleEF :
                                       SimplexBVHQuery::Method::InfoStackless;
    const unsigned long long slot[2] = {TOIMinIdentity, 0};
    toi_min_slot.copy_from(slot, 2);
    selected_counts.resize(4);

    // Validation only: repeat every detect through a fresh common query and
    // require the same candidate multisets.
    auto check_stored  = std::getenv("UIPC_CHECK_STORED_PAIRS");
    check_stored_pairs = check_stored && std::string_view{check_stored} == "1";
    if(const auto* path = std::getenv("UIPC_SIMPLEX_QUERY_TRACE_OUTPUT"); path && *path)
        query_trace = std::make_unique<simplex_query_trace::Writer>(path);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info, {engine().frame(), engine().newton_iter()});
    if(m_impl.check_stored_pairs)
        m_impl.check_detect(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.launch_filter_active(info);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    m_impl.finish_filter_active(info);
}

bool InfoStacklessBVHSimplexTrajectoryFilter::do_launch_filter_active(FilterActiveInfo& info)
{
    m_impl.launch_filter_active(info);
    return true;
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_finish_filter_active(FilterActiveInfo& info)
{
    m_impl.finish_filter_active(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

cuda_tool::CBufferView<Vector2i> InfoStacklessBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

cuda_tool::CBufferView<Vector2i> InfoStacklessBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

cuda_tool::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

cuda_tool::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}

void BVHSimplexPipeline::check_detect(DetectInfo& info)
{
    auto sorted = [](const ThisBVH::QueryBuffer& pairs)
    {
        vector<Vector2i> host(pairs.size());
        pairs.view().copy_to(host.data());
        std::ranges::sort(host,
                          [](const Vector2i& a, const Vector2i& b) {
                              return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
                          });
        return host;
    };
    auto candidates = [&]
    {
        return std::array{sorted(candidate_AllP_CodimP_pairs),
                          sorted(candidate_CodimP_AllE_pairs),
                          sorted(candidate_AllE_AllE_pairs),
                          sorted(candidate_AllP_AllT_pairs)};
    };

    auto       first           = candidates();
    const bool previous_direct = direct_queries;
    direct_queries             = true;
    try
    {
        detect(info);
    }
    catch(...)
    {
        direct_queries = previous_direct;
        throw;
    }
    direct_queries = previous_direct;
    auto second    = candidates();
    if(first != second)
    {
        throw SimEngineException(fmt::format("Stored-pair detect check failed (alpha={}): stored/queried PP {}/{}, PE {}/{}, EE {}/{}, PT {}/{}",
                                             info.alpha(),
                                             first[0].size(),
                                             second[0].size(),
                                             first[1].size(),
                                             second[1].size(),
                                             first[2].size(),
                                             second[2].size(),
                                             first[3].size(),
                                             second[3].size()));
    }
}

void BVHSimplexPipeline::detect(DetectInfo& info, simplex_query_trace::Site site)
{
    if(query_trace && !direct_queries)
        query_trace->capture(info, site);
    auto alpha                = info.alpha();
    auto Ps                   = info.positions();
    auto dxs                  = info.displacements();
    auto codimVs              = info.codim_vertices();
    auto Vs                   = info.surf_vertices();
    auto Es                   = info.surf_edges();
    auto Fs                   = info.surf_triangles();
    auto v2bs                 = info.v2b();
    auto body_self_collisions = info.body_self_collision();
    auto contact_element_ids  = info.contact_element_ids();
    auto cmts                 = info.contact_mask_tabular();

    const int nv = static_cast<int>(Ps.size());
    f32_positions.resize_discard(nv);
    f32_steps.resize_discard(nv);
    f32_ends.resize_discard(nv);
    TrajectoryPrefix prefix;
    prefix.positions     = Ps.data();
    prefix.displacements = dxs.data();
    prefix.d_hats        = info.d_hats().data();
    prefix.thicknesses   = info.thicknesses().data();
    prefix.v2bs          = v2bs.data();
    prefix.contact_ids   = contact_element_ids.data();
    prefix.vertices      = nv;
    prefix.alpha         = alpha;
    prefix.f32_positions = f32_positions.data();
    prefix.f32_steps     = f32_steps.data();
    prefix.f32_ends      = f32_ends.data();
    auto family          = [&](int                              f,
                      SizeT                            n,
                      cuda_tool::DeviceBuffer<AABB>&   aabbs,
                      cuda_tool::DeviceBuffer<IndexT>& bids,
                      cuda_tool::DeviceBuffer<IndexT>& cids)
    {
        aabbs.resize_discard(n);
        bids.resize_discard(n);
        cids.resize_discard(n);
        auto& F = prefix.family[f];
        F.n     = static_cast<int>(n);
        F.aabbs = aabbs.data();
        F.bids  = bids.data();
        F.cids  = cids.data();
    };
    family(0, codimVs.size(), codim_point_aabbs, codim_point_bids, codim_point_cids);
    family(1, Vs.size(), point_aabbs, point_bids, point_cids);
    family(2, Es.size(), edge_aabbs, edge_bids, edge_cids);
    family(3, Fs.size(), triangle_aabbs, triangle_bids, triangle_cids);
    prefix.family[0].vertices  = codimVs.data();
    prefix.family[1].vertices  = Vs.data();
    prefix.family[2].edges     = Es.data();
    prefix.family[3].triangles = Fs.data();

    auto node_pred        = NodePredicate{body_self_collisions, cmts.viewer()};
    auto allp_codimp_pred = PointPointPredicate{Vs,
                                                codimVs,
                                                Ps,
                                                dxs,
                                                info.thicknesses(),
                                                info.dimensions(),
                                                info.contact_element_ids(),
                                                info.contact_mask_tabular().viewer(),
                                                info.subscene_element_ids(),
                                                info.subscene_mask_tabular().viewer(),
                                                info.body_self_collision(),
                                                info.d_hats(),
                                                alpha,
                                                info.skin()};
    auto codimp_alle_pred = PointEdgePredicate{codimVs,
                                               Es,
                                               Ps,
                                               dxs,
                                               info.thicknesses(),
                                               info.contact_element_ids(),
                                               info.contact_mask_tabular().viewer(),
                                               info.subscene_element_ids(),
                                               info.subscene_mask_tabular().viewer(),
                                               info.body_self_collision(),
                                               info.d_hats(),
                                               alpha,
                                               info.skin()};
    auto alle_alle_pred   = EdgeEdgePredicate{Es,
                                            Ps,
                                            dxs,
                                            info.thicknesses(),
                                            info.contact_element_ids(),
                                            info.contact_mask_tabular().viewer(),
                                            info.subscene_element_ids(),
                                            info.subscene_mask_tabular().viewer(),
                                            info.body_self_collision(),
                                            info.d_hats(),
                                            alpha,
                                            info.skin()};
    auto allp_allt_pred =
        PointTrianglePredicate{Vs,
                               Fs,
                               Ps,
                               dxs,
                               info.thicknesses(),
                               info.contact_element_ids(),
                               info.contact_mask_tabular().viewer(),
                               info.subscene_element_ids(),
                               info.subscene_mask_tabular().viewer(),
                               info.body_self_collision(),
                               info.d_hats(),
                               alpha,
                               info.skin()};

    if(alpha != 0.0 && nv > 0)
    {
        alle_alle_pred.f32_positions = f32_positions.cview();
        alle_alle_pred.f32_steps     = f32_steps.cview();
        allp_allt_pred.f32_positions = f32_positions.cview();
        allp_allt_pred.f32_steps     = f32_steps.cview();
    }

    SimplexBVHQuery::CodimQueries codim{{codim_point_aabbs, codim_point_bids, codim_point_cids},
                                        allp_codimp_pred,
                                        codimp_alle_pred,
                                        &candidate_AllP_CodimP_pairs,
                                        &candidate_CodimP_AllE_pairs};
    simplex_query.force_fresh = direct_queries;
    if(!simplex_query.detect(
           lbvh_T,
           allp_allt_pred,
           alle_alle_pred,
           node_pred,
           {point_aabbs, point_bids, point_cids},
           {edge_aabbs, edge_bids, edge_cids},
           {triangle_aabbs, triangle_bids, triangle_cids},
           candidate_AllP_AllT_pairs,
           candidate_AllE_AllE_pairs,
           !direct_queries && !info.reuse_bvh_topology(),
           [&info] { info.before_read_back(); },
           &prefix,
           &codim))
        throw SimEngineException("Simplex BVH query rejected the scene topology");
}

void BVHSimplexPipeline::launch_filter_active(FilterActiveInfo& info)
{
    using namespace cuda_tool;

    auto positions = info.positions();

    SizeT N_PCoimP  = candidate_AllP_CodimP_pairs.size();
    SizeT N_CodimPE = candidate_CodimP_AllE_pairs.size();
    SizeT N_PTs     = candidate_AllP_AllT_pairs.size();
    SizeT N_EEs     = candidate_AllE_AllE_pairs.size();

    temp_PPs.resize_discard(N_PCoimP + N_CodimPE + N_PTs + N_EEs);
    temp_PEs.resize_discard(N_CodimPE + N_PTs + N_EEs);

    temp_PTs.resize_discard(N_PTs);
    temp_EEs.resize_discard(N_EEs);

    SizeT temp_PP_offset = 0;
    SizeT temp_PE_offset = 0;

    // AllP and CodimP
    if(N_PCoimP > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PCoimP);

        int n = static_cast<int>(candidate_AllP_CodimP_pairs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel;
        k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, nullptr>>>(
            positions,
            candidate_AllP_CodimP_pairs.view(),
            info.surf_vertices(),
            info.codim_vertices(),
            info.thicknesses(),
            PP_view,
            info.d_hats(),
            n);

        temp_PP_offset += N_PCoimP;
    }
    // CodimP and AllE
    if(N_CodimPE > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_CodimPE);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_CodimPE);

        int n = static_cast<int>(candidate_CodimP_AllE_pairs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel;
        k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, nullptr>>>(
            positions,
            candidate_CodimP_AllE_pairs.view(),
            info.codim_vertices(),
            info.surf_edges(),
            info.thicknesses(),
            PP_view,
            PE_view,
            info.d_hats(),
            n);

        temp_PP_offset += N_CodimPE;
        temp_PE_offset += N_CodimPE;
    }

    // AllP and AllT, AllE and AllE: one launch
    {
        auto PT_PP_view = temp_PPs.view(temp_PP_offset, N_PTs);
        auto PT_PE_view = temp_PEs.view(temp_PE_offset, N_PTs);
        temp_PP_offset += N_PTs;
        temp_PE_offset += N_PTs;
        auto EE_PP_view = temp_PPs.view(temp_PP_offset, N_EEs);
        auto EE_PE_view = temp_PEs.view(temp_PE_offset, N_EEs);
        temp_PP_offset += N_EEs;
        temp_PE_offset += N_EEs;

        int PT_n      = static_cast<int>(candidate_AllP_AllT_pairs.size());
        int EE_n      = static_cast<int>(candidate_AllE_AllE_pairs.size());
        int PT_blocks = cuda_tool::spread_grid_dim(PT_n);
        int blocks    = PT_blocks + cuda_tool::spread_grid_dim(EE_n);
        if(blocks > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k34_kernel;
            k<<<blocks, cuda_tool::spread_block_dim, 0, nullptr>>>(
                positions,
                candidate_AllP_AllT_pairs.view(),
                info.surf_vertices(),
                info.surf_triangles(),
                info.thicknesses(),
                PT_PP_view,
                PT_PE_view,
                temp_PTs.view(),
                info.d_hats(),
                PT_n,
                PT_blocks,
                positions,
                info.rest_positions(),
                candidate_AllE_AllE_pairs.view(),
                info.surf_edges(),
                info.thicknesses(),
                EE_PP_view,
                EE_PE_view,
                temp_EEs.view(),
                info.d_hats(),
                EE_n);
        }
    }

    UIPC_ASSERT(temp_PP_offset == temp_PPs.size(), "size mismatch");
    UIPC_ASSERT(temp_PE_offset == temp_PEs.size(), "size mismatch");

    {
        PPs.resize_discard(temp_PPs.size());
        PEs.resize_discard(temp_PEs.size());
        PTs.resize_discard(temp_PTs.size());
        EEs.resize_discard(temp_EEs.size());

        // One launch for the four lists; DeviceSelect::If for longer lists.
        static_assert(sizeof(Vector2i) == 2 * sizeof(IndexT)
                      && sizeof(Vector3i) == 3 * sizeof(IndexT)
                      && sizeof(Vector4i) == 4 * sizeof(IndexT));
        // The compaction stores the counts into the read-back buffer, and
        // into selected_counts for the pending active set below.
        selected_counts_readback.resize(4);
        auto list = [this](int l, auto& in, auto& out, int ints)
        {
            return CompactActiveList{reinterpret_cast<const IndexT*>(in.data()),
                                     reinterpret_cast<IndexT*>(out.data()),
                                     selected_counts_readback.device_data() + l,
                                     static_cast<int>(in.size()),
                                     ints,
                                     selected_counts.data() + l};
        };
        if(compact_active_lists({list(0, temp_PPs, PPs, 2),
                                 list(1, temp_PEs, PEs, 3),
                                 list(2, temp_PTs, PTs, 4),
                                 list(3, temp_EEs, EEs, 4)},
                                nullptr))
        {
            selected_counts_readback.enqueue_copy(nullptr);
        }
        else
        {
            DeviceSelect().If(temp_PPs.data(),
                              PPs.data(),
                              selected_counts.data(),
                              temp_PPs.size(),
                              InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PP_pred{});

            DeviceSelect().If(temp_PEs.data(),
                              PEs.data(),
                              selected_counts.data() + 1,
                              temp_PEs.size(),
                              InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PE_pred{});

            DeviceSelect().If(temp_PTs.data(),
                              PTs.data(),
                              selected_counts.data() + 2,
                              temp_PTs.size(),
                              InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT_pred{});

            DeviceSelect().If(temp_EEs.data(),
                              EEs.data(),
                              selected_counts.data() + 3,
                              temp_EEs.size(),
                              InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE_pred{});
            selected_counts_readback.copy_from(0, selected_counts.cview(), nullptr);
        }
        // read by finish_filter_active after the caller's synchronization

        // Until then, the lists at their capacity and the counts on the
        // device (selected_counts: PP, PE, PT, EE).
        PendingActiveSet pending;
        pending.PTs      = PTs.cview();
        pending.EEs      = EEs.cview();
        pending.PEs      = PEs.cview();
        pending.PPs      = PPs.cview();
        pending.PT_count = selected_counts.data() + 2;
        pending.EE_count = selected_counts.data() + 3;
        pending.PE_count = selected_counts.data() + 1;
        pending.PP_count = selected_counts.data();
        info.pending(pending);
    }
}

void BVHSimplexPipeline::finish_filter_active(FilterActiveInfo& info)
{
    {
        IndexT PP_count = selected_counts_readback[0];
        IndexT PE_count = selected_counts_readback[1];
        IndexT PT_count = selected_counts_readback[2];
        IndexT EE_count = selected_counts_readback[3];

        PPs.resize_discard(PP_count);
        PEs.resize_discard(PE_count);
        PTs.resize_discard(PT_count);
        EEs.resize_discard(EE_count);
    }

    info.PPs(PPs);
    info.PEs(PEs);
    info.PTs(PTs);
    info.EEs(EEs);

    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector2i> PPs_host;
        std::vector<Float>    PP_thicknesses_host;

        std::vector<Vector3i> PEs_host;
        std::vector<Float>    PE_thicknesses_host;

        std::vector<Vector4i> PTs_host;
        std::vector<Float>    PT_thicknesses_host;

        std::vector<Vector4i> EEs_host;
        std::vector<Float>    EE_thicknesses_host;

        PPs.copy_to(PPs_host);
        PEs.copy_to(PEs_host);
        PTs.copy_to(PTs_host);
        EEs.copy_to(EEs_host);

        std::cout << "filter result:" << std::endl;

        for(auto&& [PP, thickness] : zip(PPs_host, PP_thicknesses_host))
        {
            std::cout << "PP: " << PP.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [PE, thickness] : zip(PEs_host, PE_thicknesses_host))
        {
            std::cout << "PE: " << PE.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [PT, thickness] : zip(PTs_host, PT_thicknesses_host))
        {
            std::cout << "PT: " << PT.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [EE, thickness] : zip(EEs_host, EE_thicknesses_host))
        {
            std::cout << "EE: " << EE.transpose() << " thickness: " << thickness << "\n";
        }

        std::cout << std::flush;
    }
}

void BVHSimplexPipeline::filter_toi(FilterTOIInfo& info)
{
    using namespace cuda_tool;

    auto toi_size =
        candidate_AllP_CodimP_pairs.size() + candidate_CodimP_AllE_pairs.size()
        + candidate_AllP_AllT_pairs.size() + candidate_AllE_AllE_pairs.size();

    tois.resize_discard(toi_size);

    auto offset  = 0;
    auto PP_tois = tois.view(offset, candidate_AllP_CodimP_pairs.size());
    offset += candidate_AllP_CodimP_pairs.size();
    auto PE_tois = tois.view(offset, candidate_CodimP_AllE_pairs.size());
    offset += candidate_CodimP_AllE_pairs.size();
    auto PT_tois = tois.view(offset, candidate_AllP_AllT_pairs.size());
    offset += candidate_AllP_AllT_pairs.size();
    auto EE_tois = tois.view(offset, candidate_AllE_AllE_pairs.size());
    offset += candidate_AllE_AllE_pairs.size();

    UIPC_ASSERT(offset == toi_size, "size mismatch");

    // Every launch counts its blocks and the last block of all of them
    // finishes the minimum (toi_min.h), so the EE launch, the longest with
    // PT, runs on a side stream beside the others.
    TOIMinSlot slot{toi_min_slot.data(), toi_min_slot.data() + 1};
    const std::array<SizeT, 4> sizes{candidate_AllP_CodimP_pairs.size(),
                                     candidate_CodimP_AllE_pairs.size(),
                                     candidate_AllP_AllT_pairs.size(),
                                     candidate_AllE_AllE_pairs.size()};
    int                        last = -1;
    for(int l = 0; l < 4; ++l)
        if(sizes[l] > 0)
        {
            last = l;
            slot.blocks += cuda_tool::spread_grid_dim(static_cast<int>(sizes[l]));
        }
    auto out = [&](int) { return info.toi().data(); };
    if(!toi_side)
    {
        CUDA_TOOL_CHECK(cudaStreamCreateWithFlags(&toi_side, cudaStreamNonBlocking));
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&toi_fork, cudaEventDisableTiming));
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&toi_join, cudaEventDisableTiming));
    }
    const bool ee_beside = sizes[3] > 0;
    if(ee_beside)
    {
        CUDA_TOOL_CHECK(cudaEventRecord(toi_fork, info.stream()));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(toi_side, toi_fork, 0));
    }

    // AllP and CodimP
    {
        int n = static_cast<int>(candidate_AllP_CodimP_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel;
            k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, info.stream()>>>(
                PP_tois,
                candidate_AllP_CodimP_pairs.view(),
                info.codim_vertices(),
                info.surf_vertices(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n,
                slot,
                out(0));
        }
    }

    // CodimP and AllE
    {
        int n = static_cast<int>(candidate_CodimP_AllE_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel;
            k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, info.stream()>>>(
                PE_tois,
                candidate_CodimP_AllE_pairs.view(),
                info.codim_vertices(),
                info.thicknesses(),
                info.surf_edges(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n,
                slot,
                out(1));
        }
    }

    // AllP and AllT
    {
        int n = static_cast<int>(candidate_AllP_AllT_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel;
            k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, info.stream()>>>(
                PT_tois,
                candidate_AllP_AllT_pairs.view(),
                info.surf_vertices(),
                info.surf_triangles(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n,
                slot,
                out(2));
        }
    }

    // AllE and AllE
    {
        int n = static_cast<int>(candidate_AllE_AllE_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel;
            k<<<cuda_tool::spread_grid_dim(n), cuda_tool::spread_block_dim, 0, toi_side>>>(
                EE_tois,
                candidate_AllE_AllE_pairs.view(),
                info.surf_edges(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n,
                slot,
                out(3));
        }
    }

    if(ee_beside)
    {
        CUDA_TOOL_CHECK(cudaEventRecord(toi_join, toi_side));
        CUDA_TOOL_CHECK(cudaStreamWaitEvent(info.stream(), toi_join, 0));
    }

    // The TOI slot may be mapped host memory (a cuda_tool::ReadbackBuffer).
    if(last < 0)
        cuda_tool::store_value(info.toi().data(), large_enough_toi, info.stream());
}
}  // namespace uipc::backend::cuda
