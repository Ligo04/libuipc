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
#include <collision_detection/filters/bvh_pair_judge.h>
#include <collision_detection/filters/bvh_contact_cover.h>
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

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel(
        cuda_tool::CBufferView<IndexT>  codimVs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::BufferView<AABB>     aabbs,
        cuda_tool::CBufferView<IndexT>  v2bs,
        cuda_tool::CBufferView<IndexT>  contact_ids,
        cuda_tool::BufferView<IndexT>   bids,
        cuda_tool::BufferView<IndexT>   cids,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto vI = codimVs(i);

        Float thickness       = thicknesses(vI);
        Float d_hat_expansion = point_dcd_expansion(d_hats(vI));

        const auto& pos   = Ps(vI);
        Vector3     pos_t = pos + dxs(vI) * alpha;

        AABB aabb;
        aabb.extend(pos.cast<float>()).extend(pos_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(vI);
        cids(i)  = contact_ids(vI);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k2_kernel(
        cuda_tool::CBufferView<IndexT>  Vs,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::BufferView<AABB>     aabbs,
        cuda_tool::CBufferView<IndexT>  v2bs,
        cuda_tool::CBufferView<IndexT>  contact_ids,
        cuda_tool::BufferView<IndexT>   bids,
        cuda_tool::BufferView<IndexT>   cids,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto vI = Vs(i);

        Float thickness       = thicknesses(vI);
        Float d_hat_expansion = point_dcd_expansion(d_hats(vI));

        const auto& pos   = Ps(vI);
        Vector3     pos_t = pos + dxs(vI) * alpha;

        AABB aabb;
        aabb.extend(pos.cast<float>()).extend(pos_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(vI);
        cids(i)  = contact_ids(vI);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k3_kernel(
        cuda_tool::CBufferView<Vector2i> Es,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
        cuda_tool::CBufferView<IndexT>   v2bs,
        cuda_tool::CBufferView<IndexT>   contact_ids,
        cuda_tool::BufferView<IndexT>    bids,
        cuda_tool::BufferView<IndexT>    cids,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto eI = Es(i);

        Float thickness = edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));
        Float d_hat_expansion = edge_dcd_expansion(d_hats(eI[0]), d_hats(eI[1]));

        const auto& pos0   = Ps(eI[0]);
        const auto& pos1   = Ps(eI[1]);
        Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
        Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

        AABB aabb;

        aabb.extend(pos0.cast<float>())
            .extend(pos1.cast<float>())
            .extend(pos0_t.cast<float>())
            .extend(pos1_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(eI[0]);
        cids(i)  = contact_ids(eI[0]);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k4_kernel(
        cuda_tool::CBufferView<Vector3i> Fs,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
        cuda_tool::CBufferView<IndexT>   v2bs,
        cuda_tool::CBufferView<IndexT>   contact_ids,
        cuda_tool::BufferView<IndexT>    bids,
        cuda_tool::BufferView<IndexT>    cids,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto fI = Fs(i);

        Float thickness = triangle_thickness(
            thicknesses(fI[0]), thicknesses(fI[1]), thicknesses(fI[2]));
        Float d_hat_expansion =
            triangle_dcd_expansion(d_hats(fI[0]), d_hats(fI[1]), d_hats(fI[2]));

        const auto& pos0   = Ps(fI[0]);
        const auto& pos1   = Ps(fI[1]);
        const auto& pos2   = Ps(fI[2]);
        Vector3     pos0_t = pos0 + dxs(fI[0]) * alpha;
        Vector3     pos1_t = pos1 + dxs(fI[1]) * alpha;
        Vector3     pos2_t = pos2 + dxs(fI[2]) * alpha;

        AABB aabb;

        aabb.extend(pos0.cast<float>())
            .extend(pos1.cast<float>())
            .extend(pos2.cast<float>())
            .extend(pos0_t.cast<float>())
            .extend(pos1_t.cast<float>())
            .extend(pos2_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(fI[0]);
        cids(i)  = contact_ids(fI[0]);
    }

    // FP32 inputs of the trajectory leaf tests' first rejection
    // (swept_lower_bound_f32.h), rounded once per vertex and detect: position
    // and d_hat, step displacement (as step_forward takes it) and thickness.
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_f32_inputs_kernel(
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::CBufferView<Float>   d_hats,
        cuda_tool::CBufferView<Float>   thicknesses,
        Float                           alpha,
        cuda_tool::BufferView<float4>   positions,
        cuda_tool::BufferView<float4>   steps,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const Vector3 x = Ps(i);
        const Vector3 d = step_displacement(alpha, dxs(i));
        positions(i)    = make_float4(static_cast<float>(x.x()),
                                   static_cast<float>(x.y()),
                                   static_cast<float>(x.z()),
                                   static_cast<float>(d_hats(i)));
        steps(i)        = make_float4(static_cast<float>(d.x()),
                               static_cast<float>(d.y()),
                               static_cast<float>(d.z()),
                               static_cast<float>(thicknesses(i)));
    }

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

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_count_kernel(
        cuda_tool::CBufferView<IndexT> cids, cuda_tool::BufferView<IndexT> counts, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // The last bin collects contact ids outside the tabular.
        IndexT N = static_cast<IndexT>(counts.size()) - 1;
        IndexT c = cids(i);
        atomicAdd(&counts(c >= 0 && c < N ? c : N), 1);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_flag_kernel(
        cuda_tool::CBufferView<IndexT> cids,
        cuda_tool::CBufferView<IndexT> in_cover,
        cuda_tool::BufferView<IndexT>  ids,
        cuda_tool::BufferView<IndexT>  is_query,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // The node predicate never culls contact ids outside the tabular, so
        // such primitives always query.
        IndexT c    = cids(i);
        IndexT N    = static_cast<IndexT>(in_cover.size());
        ids(i)      = i;
        is_query(i) = (c < 0 || c >= N || in_cover(c)) ? 1 : 0;
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_gather_kernel(
        cuda_tool::CBufferView<IndexT> ids,
        cuda_tool::CBufferView<AABB>   aabbs,
        cuda_tool::CBufferView<IndexT> bids,
        cuda_tool::CBufferView<IndexT> cids,
        cuda_tool::BufferView<AABB>    out_aabbs,
        cuda_tool::BufferView<IndexT>  out_bids,
        cuda_tool::BufferView<IndexT>  out_cids,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        IndexT id    = ids(i);
        out_aabbs(i) = aabbs(id);
        out_bids(i)  = bids(id);
        out_cids(i)  = cids(id);
    }

    // slots(ids(i)) = i over the gathered range of a split (the rest stays -1).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_slots_kernel(
        cuda_tool::CBufferView<IndexT> ids, cuda_tool::BufferView<IndexT> slots, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        slots(ids(i)) = i;
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_pairs_kernel(
        cuda_tool::CBufferView<Vector2i> query_pairs,
        cuda_tool::CBufferView<IndexT>   query_ids,
        cuda_tool::CBufferView<IndexT>   leaf_ids,
        bool                             ordered,
        cuda_tool::BufferView<Vector2i>  pairs,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // (compacted query, leaf) -> primitive ids. An empty leaf map means the
        // leaves already are primitive ids; EE keeps the self-join's (min, max).
        Vector2i p = query_pairs(i);
        IndexT   a = query_ids(p[0]);
        IndexT   b = leaf_ids.size() > 0 ? leaf_ids(p[1]) : p[1];
        pairs(i)   = (ordered && b < a) ? Vector2i{b, a} : Vector2i{a, b};
    }

    // cover_remap_pairs_kernel on the first *count pairs of `in`, written to
    // `out` from *offset on (null: 0). The counts are the judge scan's on the
    // device, so the remaps of a judged detect follow its scatter before the
    // host reads them back: one launch, a grid-stride loop per segment.
    struct InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_segment
    {
        const Vector2i* in;
        const IndexT*   query_ids;
        const IndexT*   leaf_ids;  // null: the leaves already are primitive ids
        bool            ordered;
        Vector2i*       out;
        const int*      count;
        const int*      offset;
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_segments
    {
        InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_segment segment[3];
        int segments = 0;
    };

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_judged_kernel(
        InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_segments remaps)
    {
        const int stride = static_cast<int>(gridDim.x * blockDim.x);
        for(int s = 0; s < remaps.segments; ++s)
        {
            const auto& S  = remaps.segment[s];
            const int   n  = *S.count;
            const int   at = S.offset ? *S.offset : 0;
            for(int i = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); i < n;
                i += stride)
            {
                Vector2i p = S.in[i];
                IndexT   a = S.query_ids[p[0]];
                IndexT   b = S.leaf_ids ? S.leaf_ids[p[1]] : p[1];
                S.out[at + i] = (S.ordered && b < a) ? Vector2i{b, a} : Vector2i{a, b};
            }
        }
    }

    /****************************************************
    *                  Stored pairs
    ****************************************************/

    // Skins of the stored pairs, relative to d_hat. Any skin, also a different
    // one per primitive, is exact (each box is checked against its own grown
    // copy); a larger one stores more pairs and refreshes less often. Without a
    // contact cover every primitive takes the uniform skin. With one, the
    // cover's primitives (the ones that query; on X_square the parcels) take it
    // too, or the thick skin while their boxes keep leaving the uniform one
    // within a frame, and the others (on X_square the belts, the robot and the
    // static geometry, whose boxes mostly stay put within a frame) take the
    // thin skin, or the uniform one while they keep leaving it. The level of
    // each group follows the previous frame (see stored_cover_thick).
    constexpr Float StoredSkinOverDHat      = 1.0;
    constexpr Float StoredThickSkinOverDHat = 2.0;
    constexpr Float StoredThinSkinOverDHat  = 0.01;

    // A detect that may rebuild the BVH topology (every frame's first)
    // rebuilds it only every TopologyRebuildPeriod-th time and refits the kept
    // trees and query orders otherwise. Any topology and query order gives the
    // same candidate multisets; a kept one only fits the moved boxes looser.
    constexpr SizeT TopologyRebuildPeriod = 8;

    // ids: the primitive of each box (empty: box i); is_query: per primitive,
    // in the cover (empty: every box takes `skin`).
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_grow_aabbs_kernel(
        cuda_tool::CBufferView<AABB>   aabbs,
        cuda_tool::CBufferView<IndexT> ids,
        cuda_tool::CBufferView<IndexT> is_query,
        float                          skin,
        float                          other_skin,
        cuda_tool::BufferView<AABB>    grown,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        IndexT p   = ids.size() > 0 ? ids(i) : i;
        float  s   = is_query.size() == 0 || is_query(p) ? skin : other_skin;
        AABB   box = aabbs(i);
        box.min().array() -= s;
        box.max().array() += s;
        grown(i) = box;
    }

    // Flags any box that is not inside its grown copy (escaped(0), NaN
    // included). With a cover split (is_query), also flags a box outside its
    // grown copy shrunk to the next lower skin level of its group: a cover box
    // by cover_shrink (escaped(1)), another box by other_shrink (escaped(2));
    // a zero shrink flags the box's own escape.
    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_contained_kernel(
        cuda_tool::CBufferView<AABB>   aabbs,
        cuda_tool::CBufferView<AABB>   grown,
        cuda_tool::CBufferView<IndexT> is_query,
        float                          cover_shrink,
        float                          other_shrink,
        cuda_tool::BufferView<IndexT>  escaped,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        const AABB& box    = aabbs(i);
        const AABB& g      = grown(i);
        bool        inside = (g.min().array() <= box.min().array()).all()
                      && (box.max().array() <= g.max().array()).all();
        if(!inside)
            escaped(0) = 1;
        if(is_query.size() == 0)
            return;
        const bool  cover = is_query(i) != 0;
        const float s     = cover ? cover_shrink : other_shrink;
        bool        lower = ((g.min().array() + s) <= box.min().array()).all()
                     && (box.max().array() <= (g.max().array() - s)).all();
        if(!lower)
            escaped(cover ? 1 : 2) = 1;
    }

    using namespace bvh_pair_judge;
    using InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue = bvh_pair_judge::EEQueue;
    using InfoStacklessBVHSimplexTrajectoryFilter_judged_list = bvh_pair_judge::PairList;
    using InfoStacklessBVHSimplexTrajectoryFilter_judged_lists = bvh_pair_judge::PairLists;
    // Weighted minimum vertex cover of the enabled contact-tabular graph
    // (row-major N x N mask, symmetric). Elements without query primitives
    // weigh nothing and always join S, so S stays a cover if they gain
    // primitives later; self-contact elements must be in S. Exhaustive over up
    // to 20 remaining elements, greedy (least weight per uncovered edge) above.


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

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_collect_query_counts_kernel(
        cuda_tool::CVarView<IndexT>    allp_codimp_count,
        cuda_tool::CVarView<IndexT>    codimp_alle_count,
        cuda_tool::CVarView<IndexT>    alle_alle_count,
        cuda_tool::CVarView<IndexT>    allp_allt_count,
        cuda_tool::CVarView<IndexT>    allp_covert_count,
        cuda_tool::CBufferView<IndexT> escaped,
        cuda_tool::BufferView<IndexT>  counts)
    {
        if(blockIdx.x != 0 || threadIdx.x != 0)
            return;
        counts(0) = *allp_codimp_count;
        counts(1) = *codimp_alle_count;
        counts(2) = *alle_alle_count;
        counts(3) = *allp_allt_count;
        counts(4) = *allp_covert_count;
        counts(5) = escaped(0);
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

    // Validation only: UIPC_BVH_QUERY_CHECK=<file> reruns every Other query
    // with the per-lane reference traversal after each detect and appends the
    // comparison of the pair multisets to <file> (JSON lines).
    FILE* query_check_log()
    {
        static FILE* f = []() -> FILE*
        {
            const char* p = std::getenv("UIPC_BVH_QUERY_CHECK");
            return (p && *p) ? std::fopen(p, "a") : nullptr;
        }();
        return f;
    }

    // Pairs (with multiplicity) held by only one of the two lists.
    std::array<SizeT, 2> pair_multiset_difference(vector<Vector2i> a, vector<Vector2i> b)
    {
        auto less = [](const Vector2i& l, const Vector2i& r)
        { return l[0] < r[0] || (l[0] == r[0] && l[1] < r[1]); };
        std::sort(a.begin(), a.end(), less);
        std::sort(b.begin(), b.end(), less);
        SizeT i = 0, j = 0, only_a = 0, only_b = 0;
        while(i < a.size() && j < b.size())
        {
            if(less(a[i], b[j]))
                ++only_a, ++i;
            else if(less(b[j], a[i]))
                ++only_b, ++j;
            else
                ++i, ++j;
        }
        return {only_a + (a.size() - i), only_b + (b.size() - j)};
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
    query_counts.resize(8);
    const unsigned long long slot[2] = {TOIMinIdentity, 0};
    toi_min_slot.copy_from(slot, 2);
    selected_counts.resize(4);
    stored_escaped.resize(3);
    judge_ee_queue_counts.resize(3);  // zeros
    int device = 0;
    CUDA_TOOL_CHECK(cudaGetDevice(&device));
    CUDA_TOOL_CHECK(cudaDeviceGetAttribute(&judge_sm_count, cudaDevAttrMultiProcessorCount, device));

    // Validation only: every detect is repeated with all chains on the
    // default stream and the queries instead of the stored pairs, and must
    // give the same candidate multisets.
    auto check_stored  = std::getenv("UIPC_CHECK_STORED_PAIRS");
    check_stored_pairs = check_stored && std::string_view{check_stored} == "1";
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info, true);
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

bool BVHSimplexPipeline::cover_unchanged(DetectInfo& info) const
{
    auto                 cmts  = info.contact_mask_tabular();
    std::array<SizeT, 3> sizes = {
        edge_cids.size(), point_cids.size(), triangle_cids.size()};
    if(cmts.data() != cover_tabular || cmts.extent().h != cover_tabular_size || sizes != cover_sizes)
        return false;
    return !(cover_EE_usable || cover_PT_usable) || cover_split_ready;
}

bool BVHSimplexPipeline::update_cover(DetectInfo& info, bool with_gather)
{
    cover_EE = false;
    cover_PT = false;

    auto                 cmts  = info.contact_mask_tabular();
    SizeT                N     = cmts.extent().h;
    std::array<SizeT, 3> sizes = {
        edge_cids.size(), point_cids.size(), triangle_cids.size()};
    if(cmts.data() != cover_tabular || N != cover_tabular_size || sizes != cover_sizes)
    {
        cover_tabular      = cmts.data();
        cover_tabular_size = N;
        cover_sizes        = sizes;
        cover_EE_usable    = false;
        cover_PT_usable    = false;
        cover_split_ready  = false;

        // Without a readable symmetric tabular the original queries stay.
        bool readable = cmts.data() != nullptr && N > 0 && cmts.extent().w == N;
        vector<IndexT> mask(readable ? N * N : 0);
        if(readable)
            cuda_tool::CBufferView<IndexT>{cmts.data(), N * N}.copy_to(mask.data());
        for(SizeT a = 0; a < N && readable; ++a)
            for(SizeT b = 0; b < a && readable; ++b)
                readable = (mask[a * N + b] != 0) == (mask[b * N + a] != 0);
        stored_enabled = readable;
        stored_valid   = false;
        if(!readable)
        {
            logger::warn("Contact-cover queries: contact tabular unusable, keeping the original EE/PT queries");
            return false;
        }

        // Query primitives per contact element (last bin: ids outside the tabular).
        cover_counts.resize_discard(2 * (N + 1));
        cuda_tool::BufferLaunch().fill(cover_counts.view(), 0);
        auto count = [](cuda_tool::CBufferView<IndexT> cids, cuda_tool::BufferView<IndexT> bins)
        {
            int n = static_cast<int>(cids.size());
            if(n == 0)
                return;
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_cover_count_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                cids, bins, n);
        };
        count(edge_cids, cover_counts.view(0, N + 1));
        count(point_cids, cover_counts.view(N + 1, N + 1));
        vector<IndexT> weights(2 * (N + 1));
        cover_counts.copy_to(weights.data());

        auto choose = [&](const IndexT* w, cuda_tool::DeviceBuffer<IndexT>& cover, const char* family)
        {
            auto        in_cover = contact_vertex_cover(mask, N, w);
            int64_t     total    = 0;
            int64_t     covered  = w[N];
            std::string elements;
            for(SizeT c = 0; c <= N; ++c)
                total += w[c];
            for(SizeT c = 0; c < N; ++c)
            {
                if(!in_cover[c])
                    continue;
                covered += w[c];
                if(w[c] > 0)
                    elements += (elements.empty() ? "" : ",") + std::to_string(c);
            }
            cover.copy_from(in_cover.data(), N);
            bool usable = total > 0 && covered * 5 < total * 3;
            logger::info("Contact-cover queries {}: S = {{{}}} holds {} of {} query primitives -> {} queries",
                         family,
                         elements,
                         covered,
                         total,
                         usable ? "cover" : "original");
            return usable;
        };
        cover_EE_usable = choose(weights.data(), edge_cover, "EE");
        cover_PT_usable = choose(weights.data() + N + 1, point_cover, "PT");
    }

    cover_EE = cover_EE_usable;
    cover_PT = cover_PT_usable;
    if(!cover_EE && !cover_PT)
        return false;

    // Split the primitives by S once per S. The split depends only on S and
    // on the primitives' contact ids, which the vertex reporters write at
    // init (later frames rewrite the same ids); a changed primitive count or
    // tabular clears cover_split_ready above.
    bool refresh = !cover_split_ready;
    if(refresh)
    {
        cover_counts.resize_discard(3);
        cuda_tool::BufferLaunch().fill(cover_counts.view(), 0);
        auto split = [this](CoverSplit&                    s,
                            cuda_tool::CBufferView<IndexT> cids,
                            cuda_tool::CBufferView<IndexT> in_cover,
                            IndexT*                        count)
        {
            int n = static_cast<int>(cids.size());
            s.is_query.resize_discard(n);
            s.ids.resize_discard(n);
            cover_scratch.resize_discard(n);
            if(n == 0)
                return;
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_cover_flag_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                cids, in_cover, cover_scratch.view(), s.is_query.view(), n);
            cuda_tool::DevicePartition().Flagged(
                cover_scratch.data(), s.is_query.data(), s.ids.data(), count, n);
        };
        if(cover_EE)
            split(cover_edges, edge_cids, edge_cover, cover_counts.data());
        if(cover_PT)
        {
            split(cover_points, point_cids, point_cover, cover_counts.data() + 1);
            split(cover_triangles, triangle_cids, point_cover, cover_counts.data() + 2);
        }
        std::array<IndexT, 3> counts{};
        cover_counts.copy_to(counts.data());
        cover_edges.count     = counts[0];
        cover_points.count    = counts[1];
        cover_triangles.count = counts[2];
        cover_split_ready     = true;

        // Each primitive's index in the gathered lists below, for the
        // prefix kernel's gather.
        auto invert = [](CoverSplit& s, SizeT n, SizeT gathered)
        {
            s.slots.resize_discard(n);
            if(n == 0)
                return;
            cuda_tool::BufferLaunch().fill(s.slots.view(), static_cast<IndexT>(-1));
            int m = static_cast<int>(gathered);
            if(m == 0)
                return;
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_cover_slots_kernel;
            k<<<cuda_tool::best_grid_dim(m, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                s.ids.view(0, gathered), s.slots.view(), m);
        };
        if(cover_EE)
            invert(cover_edges, edge_cids.size(), cover_edges.count);
        if(cover_PT)
        {
            invert(cover_points, point_cids.size(), point_cids.size());
            invert(cover_triangles, triangle_cids.size(), cover_triangles.count);
        }
    }
    if(!with_gather)
    {
        UIPC_ASSERT(!refresh, "A refreshed cover split needs the gather");
        return refresh;
    }

    // Gather the query AABBs and IDs in split order (points: S_P, then the rest).
    auto gather = [](CoverSplit&                    s,
                     cuda_tool::CBufferView<AABB>   aabbs,
                     cuda_tool::CBufferView<IndexT> bids,
                     cuda_tool::CBufferView<IndexT> cids,
                     SizeT                          count)
    {
        s.aabbs.resize_discard(count);
        s.bids.resize_discard(count);
        s.cids.resize_discard(count);
        int n = static_cast<int>(count);
        if(n == 0)
            return;
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_cover_gather_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            s.ids.view(0, count),
            aabbs,
            bids,
            cids,
            s.aabbs.view(),
            s.bids.view(),
            s.cids.view(),
            n);
    };
    if(cover_EE)
        gather(cover_edges, edge_aabbs, edge_bids, edge_cids, cover_edges.count);
    if(cover_PT)
    {
        gather(cover_points, point_aabbs, point_bids, point_cids, point_cids.size());
        gather(cover_triangles,
               triangle_aabbs,
               triangle_bids,
               triangle_cids,
               cover_triangles.count);
    }
    return refresh;
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

    auto first     = candidates();
    direct_queries = true;
    detect(info, false);
    direct_queries = false;
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

void BVHSimplexPipeline::detect(DetectInfo& info, bool concurrent)
{
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

    // Each box grows by its primitive's full d_hat + thickness, so the boxes of
    // a pair overlap whenever it comes within d_hat + thickness plus the pair
    // d_hat. That slack exceeds DetectInfo::skin() (at most d_hat / 2, see
    // GlobalTrajectoryFilter::reuse_candidates), which therefore widens only
    // the leaf tests.
    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());
    point_bids.resize(Vs.size());
    triangle_bids.resize(Fs.size());
    edge_bids.resize(Es.size());
    point_cids.resize(Vs.size());
    triangle_cids.resize(Fs.size());
    edge_cids.resize(Es.size());

    // While the cover split stays, one prefix of two launches
    // (vertex_prep_kernel, prefix_kernel) writes the boxes, the cover
    // gathers, the FP32 inputs and the stored pairs' containment flags, bit
    // for bit as the separate kernels of the other path.
    const bool fused_prefix = single_ef || cover_unchanged(info);

    // build AABBs for codim vertices
    if(codimVs.size() > 0)
    {
        codim_point_aabbs.resize(codimVs.size());
        codim_point_bids.resize(codimVs.size());
        codim_point_cids.resize(codimVs.size());

        int  n = static_cast<int>(codimVs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel;
        if(!fused_prefix)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                codimVs,
                Ps,
                dxs,
                codim_point_aabbs.view(),
                v2bs,
                contact_element_ids,
                codim_point_bids.view(),
                codim_point_cids.view(),
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
    }

    // build AABBs for surf vertices (including codim vertices)
    {
        int n = static_cast<int>(Vs.size());
        if(!fused_prefix && n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k2_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Vs,
                dxs,
                Ps,
                point_aabbs.view(),
                v2bs,
                contact_element_ids,
                point_bids.view(),
                point_cids.view(),
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    // build AABBs for edges
    {
        int n = static_cast<int>(Es.size());
        if(!fused_prefix && n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k3_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Es,
                Ps,
                edge_aabbs.view(),
                v2bs,
                contact_element_ids,
                edge_bids.view(),
                edge_cids.view(),
                dxs,
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    // build AABBs for triangles
    {
        int n = static_cast<int>(Fs.size());
        if(!fused_prefix && n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k4_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Fs,
                Ps,
                triangle_aabbs.view(),
                v2bs,
                contact_element_ids,
                triangle_bids.view(),
                triangle_cids.view(),
                dxs,
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    // A refreshed split changes the cover-tree leaves and the query lists
    // (fused_prefix: update_cover sets cover_EE and cover_PT only).
    bool split_refreshed = !single_ef && update_cover(info, !fused_prefix);
    bool keep_topology   = info.reuse_bvh_topology();
    if(!keep_topology)
        keep_topology = ++topology_requests % TopologyRebuildPeriod != 0;
    bool reuse_cover = !split_refreshed && keep_topology;

    std::array<SizeT, 4> sizes{codimVs.size(), Vs.size(), Es.size(), Fs.size()};
    bool                 refresh = !stored_valid || !info.reuse_bvh_topology()
                   || split_refreshed || sizes != stored_sizes;
    // The stored pairs are judged first: check every box against its grown copy.
    const bool check_contained = stored_enabled && !direct_queries && !refresh;
    // the next lower skin level of each group, as a shrink
    const Float d_hat = info.d_hat();
    const float cover_shrink =
        stored_cover_thick ?
            static_cast<float>((StoredThickSkinOverDHat - StoredSkinOverDHat) * d_hat) :
            0.0f;
    const float other_shrink =
        stored_other_uniform ?
            static_cast<float>((StoredSkinOverDHat - StoredThinSkinOverDHat) * d_hat) :
            0.0f;

    TrajectoryPrefix prefix;
    if(fused_prefix)
    {
        int nv = static_cast<int>(Ps.size());
        f32_positions.resize_discard(nv);
        f32_steps.resize_discard(nv);
        f32_ends.resize_discard(nv);
        // The gathered lists keep their split's sizes.
        auto gathered = [](CoverSplit& s, SizeT count)
        {
            s.aabbs.resize_discard(count);
            s.bids.resize_discard(count);
            s.cids.resize_discard(count);
        };
        if(cover_EE)
            gathered(cover_edges, cover_edges.count);
        if(cover_PT)
        {
            gathered(cover_points, point_cids.size());
            gathered(cover_triangles, cover_triangles.count);
        }

        prefix.accepted_step_endpoints = single_ef;
        prefix.positions               = Ps.data();
        prefix.displacements           = dxs.data();
        prefix.d_hats                  = info.d_hats().data();
        prefix.thicknesses             = info.thicknesses().data();
        prefix.v2bs                    = v2bs.data();
        prefix.contact_ids             = contact_element_ids.data();
        prefix.vertices                = nv;
        prefix.alpha                   = alpha;
        prefix.f32_positions           = f32_positions.data();
        prefix.f32_steps               = f32_steps.data();
        prefix.f32_ends                = f32_ends.data();
        prefix.escaped      = check_contained ? stored_escaped.data() : nullptr;
        prefix.cover_shrink = cover_shrink;
        prefix.other_shrink = other_shrink;
        auto family         = [&](TrajectoryPrefixFamily&          F,
                          SizeT                            n,
                          cuda_tool::DeviceBuffer<AABB>&   aabbs,
                          cuda_tool::DeviceBuffer<IndexT>& bids,
                          cuda_tool::DeviceBuffer<IndexT>& cids,
                          CoverSplit*                      split,
                          cuda_tool::DeviceBuffer<AABB>&   grown)
        {
            F.n     = static_cast<int>(n);
            F.aabbs = aabbs.data();
            F.bids  = bids.data();
            F.cids  = cids.data();
            if(split)
            {
                F.slots          = split->slots.data();
                F.gathered_aabbs = split->aabbs.data();
                F.gathered_bids  = split->bids.data();
                F.gathered_cids  = split->cids.data();
            }
            if(check_contained)
            {
                F.grown    = grown.data();
                F.is_query = split ? split->is_query.data() : nullptr;
            }
        };
        prefix.family[0].vertices  = codimVs.data();
        prefix.family[1].vertices  = Vs.data();
        prefix.family[2].edges     = Es.data();
        prefix.family[3].triangles = Fs.data();
        family(prefix.family[0],
               codimVs.size(),
               codim_point_aabbs,
               codim_point_bids,
               codim_point_cids,
               nullptr,
               grown_codim_point_aabbs);
        family(prefix.family[1],
               Vs.size(),
               point_aabbs,
               point_bids,
               point_cids,
               cover_PT ? &cover_points : nullptr,
               grown_point_aabbs);
        family(prefix.family[2],
               Es.size(),
               edge_aabbs,
               edge_bids,
               edge_cids,
               cover_EE ? &cover_edges : nullptr,
               grown_edge_aabbs);
        family(prefix.family[3],
               Fs.size(),
               triangle_aabbs,
               triangle_bids,
               triangle_cids,
               cover_PT ? &cover_triangles : nullptr,
               grown_triangle_aabbs);
        if(!single_ef)
            launch_trajectory_prefix(prefix, nullptr);
    }

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

    // Trajectory leaf tests reject most far pairs on FP32 inputs first.
    if(alpha != 0.0 && Ps.size() > 0)
    {
        if(!fused_prefix)
        {
            int n = static_cast<int>(Ps.size());
            f32_positions.resize_discard(n);
            f32_steps.resize_discard(n);
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_f32_inputs_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Ps,
                dxs,
                info.d_hats(),
                info.thicknesses(),
                alpha,
                f32_positions.view(),
                f32_steps.view(),
                n);
        }
        alle_alle_pred.f32_positions = f32_positions.view();
        alle_alle_pred.f32_steps     = f32_steps.view();
        allp_allt_pred.f32_positions = f32_positions.view();
        allp_allt_pred.f32_steps     = f32_steps.view();
    }

    if(single_ef)
    {
        single_ef_query.force_fresh = direct_queries;
        if(single_ef_query.detect(
               lbvh_T,
               allp_allt_pred,
               alle_alle_pred,
               node_pred,
               {point_aabbs, point_bids, point_cids},
               {edge_aabbs, edge_bids, edge_cids},
               {triangle_aabbs, triangle_bids, triangle_cids},
               candidate_AllP_AllT_pairs,
               candidate_AllE_AllE_pairs,
               !info.reuse_bvh_topology(),
               [&info] { info.before_read_back(); },
               &prefix))
        {
            // PP/PE retain the codimensional path. PT/EE share one EF tree.
            candidate_AllP_CodimP_pairs.m_size = 0;
            candidate_CodimP_AllE_pairs.m_size = 0;
            candidate_AllP_CodimP_pairs.m_cpNum.copy_from(0);
            candidate_CodimP_AllE_pairs.m_cpNum.copy_from(0);
            if(codimVs.size() > 0)
            {
                if(!keep_topology
                   || !lbvh_CodimP.refit(codim_point_aabbs, codim_point_bids, codim_point_cids))
                    lbvh_CodimP.build(codim_point_aabbs, codim_point_bids, codim_point_cids);
                if(!keep_topology || !lbvh_E.refit(edge_aabbs, edge_bids, edge_cids))
                    lbvh_E.build(edge_aabbs, edge_bids, edge_cids);
                lbvh_CodimP.query(point_aabbs,
                                  point_bids,
                                  point_cids,
                                  cmts,
                                  node_pred,
                                  allp_codimp_pred,
                                  candidate_AllP_CodimP_pairs,
                                  keep_topology);
                lbvh_E.query(codim_point_aabbs,
                             codim_point_bids,
                             codim_point_cids,
                             cmts,
                             node_pred,
                             codimp_alle_pred,
                             candidate_CodimP_AllE_pairs,
                             keep_topology);
            }
            return;
        }
    }

    // Runs every query and downloads the queue counts. With grown == true the
    // trees and queries take the grown boxes, the leaf predicates keep their
    // position-free checks only and the pairs go to the stored lists;
    // otherwise the current boxes and full predicates fill the candidates.
    auto run_queries = [&](bool grown)
    {
        using AABBs = cuda_tool::DeviceBuffer<AABB>;
        auto pick   = [grown](AABBs& now, AABBs& g) -> AABBs&
        { return grown ? g : now; };
        auto& P_aabbs  = pick(point_aabbs, grown_point_aabbs);
        auto& CP_aabbs = pick(codim_point_aabbs, grown_codim_point_aabbs);
        auto& E_aabbs  = pick(edge_aabbs, grown_edge_aabbs);
        auto& T_aabbs  = pick(triangle_aabbs, grown_triangle_aabbs);
        auto& SE_aabbs = pick(cover_edges.aabbs, cover_edges.grown_aabbs);
        auto& SP_aabbs = pick(cover_points.aabbs, cover_points.grown_aabbs);
        auto& ST_aabbs = pick(cover_triangles.aabbs, cover_triangles.grown_aabbs);

        auto& out_PCP = grown ? stored_AllP_CodimP_pairs : candidate_AllP_CodimP_pairs;
        auto& out_CPE = grown ? stored_CodimP_AllE_pairs : candidate_CodimP_AllE_pairs;
        auto& out_EE = grown ? stored_AllE_AllE_pairs : candidate_AllE_AllE_pairs;
        auto& out_PT = grown ? stored_AllP_AllT_pairs : candidate_AllP_AllT_pairs;
        auto& out_PcT = grown ? stored_AllP_CoverT_pairs : candidate_AllP_CoverT_pairs;

        auto pcp_pred     = allp_codimp_pred;
        auto cpe_pred     = codimp_alle_pred;
        auto ee_pred      = alle_alle_pred;
        auto pt_pred      = allp_allt_pred;
        pcp_pred.geometry = !grown;
        cpe_pred.geometry = !grown;
        ee_pred.geometry  = !grown;
        pt_pred.geometry  = !grown;

        // Three independent chains (tree refit or build, then its queries) only
        // read the AABBs and IDs written above: lbvh_E with the CodimP-E and EE
        // queries on the default stream, lbvh_T with the PT query and
        // lbvh_T_cover with the cover PT query on their own streams. Both join
        // the default stream before the query counts are collected.
        auto         default_stream = cuda_tool::default_stream();
        bool         fork_cover     = concurrent && cover_PT;
        cudaStream_t pt_s = concurrent ? pt_stream.fork() : default_stream;
        cudaStream_t cover_pt_s = fork_cover ? cover_pt_stream.fork() : default_stream;

        if(!keep_topology || !lbvh_E.refit(E_aabbs, edge_bids, edge_cids))
            lbvh_E.build(E_aabbs, edge_bids, edge_cids);
        if(!keep_topology || !lbvh_T.refit(T_aabbs, triangle_bids, triangle_cids, pt_s))
            lbvh_T.build(T_aabbs, triangle_bids, triangle_cids, pt_s);
        if(cover_PT)
        {
            auto& T = cover_triangles;
            if(!reuse_cover || !lbvh_T_cover.refit(ST_aabbs, T.bids, T.cids, cover_pt_s))
                lbvh_T_cover.build(ST_aabbs, T.bids, T.cids, cover_pt_s);
        }

        auto launch_allp_codimp = [&](bool rebuild_query)
        {
            lbvh_CodimP.launch_query(
                P_aabbs, point_bids, point_cids, cmts, node_pred, pcp_pred, out_PCP, rebuild_query, keep_topology);
        };
        auto launch_codimp_alle = [&](bool rebuild_query)
        {
            lbvh_E.launch_query(CP_aabbs,
                                codim_point_bids,
                                codim_point_cids,
                                cmts,
                                node_pred,
                                cpe_pred,
                                out_CPE,
                                rebuild_query,
                                keep_topology);
        };
        auto launch_alle_alle = [&](bool rebuild_query)
        {
            if(!cover_EE)
            {
                lbvh_E.launch_detect(cmts, node_pred, ee_pred, out_EE);
                return;
            }
            auto& E = cover_edges;
            lbvh_E.launch_query(SE_aabbs,
                                E.bids,
                                E.cids,
                                cmts,
                                node_pred,
                                InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred{
                                    E.ids.view(0, E.count), E.is_query.view(), ee_pred},
                                out_EE,
                                rebuild_query,
                                reuse_cover);
        };
        auto launch_allp_allt = [&](bool rebuild_query, cudaStream_t stream)
        {
            if(!cover_PT)
            {
                lbvh_T.launch_query(
                    P_aabbs, point_bids, point_cids, cmts, node_pred, pt_pred, out_PT, rebuild_query, keep_topology, stream);
                return;
            }
            auto& P = cover_points;
            lbvh_T.launch_query(SP_aabbs.view(0, P.count),
                                P.bids.view(0, P.count),
                                P.cids.view(0, P.count),
                                cmts,
                                node_pred,
                                InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred{
                                    P.ids.view(0, P.count), {}, pt_pred},
                                out_PT,
                                rebuild_query,
                                reuse_cover,
                                stream);
        };
        // Points outside S_P can only meet the triangles of S_P.
        auto launch_allp_covert = [&](bool rebuild_query, cudaStream_t stream)
        {
            auto& P = cover_points;
            auto& T = cover_triangles;
            lbvh_T_cover.launch_query(
                SP_aabbs.view(P.count),
                P.bids.view(P.count),
                P.cids.view(P.count),
                cmts,
                node_pred,
                InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred{
                    P.ids.view(P.count), T.ids.view(0, T.count), pt_pred},
                out_PcT,
                rebuild_query,
                reuse_cover,
                stream);
        };

        if(codimVs.size() > 0)
        {
            if(!keep_topology || !lbvh_CodimP.refit(CP_aabbs, codim_point_bids, codim_point_cids))
                lbvh_CodimP.build(CP_aabbs, codim_point_bids, codim_point_cids);
            launch_allp_codimp(true);
        }
        else
        {
            out_PCP.m_cpNum.fill(0);
        }
        launch_codimp_alle(true);
        launch_alle_alle(true);
        launch_allp_allt(true, pt_s);
        if(cover_PT)
            launch_allp_covert(true, cover_pt_s);
        else
            out_PcT.m_cpNum.fill(0);
        if(concurrent)
            pt_stream.join();
        if(fork_cover)
            cover_pt_stream.join();

        InfoStacklessBVHSimplexTrajectoryFilter_collect_query_counts_kernel<<<1, 1>>>(
            out_PCP.m_cpNum.cview(),
            out_CPE.m_cpNum.cview(),
            out_EE.m_cpNum.cview(),
            out_PT.m_cpNum.cview(),
            out_PcT.m_cpNum.cview(),
            stored_escaped.view(),
            query_counts.view());

        std::array<IndexT, 8> host_counts{};
        query_counts.copy_to(host_counts.data());
        IndexT covert_count = host_counts[4];

        if(lbvh_CodimP.prepare_query_result(out_PCP, host_counts[0]))
            launch_allp_codimp(false);
        if(lbvh_E.prepare_query_result(out_CPE, host_counts[1]))
            launch_codimp_alle(false);
        if(lbvh_E.prepare_query_result(out_EE, host_counts[2]))
            launch_alle_alle(false);
        // The cover-tree PT candidates are appended behind the S_P ones. Reruns
        // stay on the default stream.
        if(lbvh_T.prepare_query_result(out_PT, host_counts[3] + (grown ? 0 : covert_count)))
            launch_allp_allt(false, default_stream);
        if(cover_PT && lbvh_T_cover.prepare_query_result(out_PcT, covert_count))
            launch_allp_covert(false, default_stream);

        // UIPC_BVH_QUERY_CHECK: every query run (candidate queries and
        // stored-pair refreshes alike) is repeated with the per-lane kernel.
        if(FILE* log = query_check_log())
        {
            // Rerun each query in place with the reference traversal, compare,
            // then restore the cooperative output the step continues with.
            static SizeT detect_index = 0;
            bool         same         = true;
            std::string  queries;
            auto         check = [&](const char*                    name,
                             InfoStacklessBVH&              bvh,
                             InfoStacklessBVH::QueryBuffer& q,
                             IndexT                         count,
                             auto&&                         relaunch)
            {
                vector<Vector2i> cooperative(count);
                if(count > 0)
                    q.m_pairs.view(0, count).copy_to(cooperative.data());
                bvh.config().warp_cooperative_query = false;
                relaunch(false);
                int ref_count = q.m_cpNum;
                if(static_cast<SizeT>(ref_count) > q.m_pairs.size())
                {
                    q.reserve(ref_count);
                    relaunch(false);
                }
                bvh.config().warp_cooperative_query = true;
                vector<Vector2i> reference(ref_count);
                if(ref_count > 0)
                    q.m_pairs.view(0, ref_count).copy_to(reference.data());
                if(count > 0)
                    q.m_pairs.view(0, count).copy_from(cooperative.data());
                q.m_cpNum = count;

                auto diff = pair_multiset_difference(cooperative, reference);
                same      = same && diff[0] == 0 && diff[1] == 0;
                queries += fmt::format(R"({}"{}":[{},{},{},{}])",
                                       queries.empty() ? "" : ",",
                                       name,
                                       count,
                                       ref_count,
                                       diff[0],
                                       diff[1]);
            };
            if(codimVs.size() > 0)
                check("PP", lbvh_CodimP, out_PCP, host_counts[0], launch_allp_codimp);
            check("PE", lbvh_E, out_CPE, host_counts[1], launch_codimp_alle);
            check("EE", lbvh_E, out_EE, host_counts[2], launch_alle_alle);
            check("PT",
                  lbvh_T,
                  out_PT,
                  host_counts[3],
                  [&](bool r) { launch_allp_allt(r, default_stream); });
            if(cover_PT)
                check("PcoverT",
                      lbvh_T_cover,
                      out_PcT,
                      covert_count,
                      [&](bool r) { launch_allp_covert(r, default_stream); });
            if(!same)
                logger::error("BVH warp-cooperative query check mismatch at query run {}: {}",
                              detect_index,
                              queries);
            // Per query: [cooperative, reference, only cooperative, only reference].
            std::fprintf(log,
                         "{\"detect\":%zu,\"alpha\":%.9g,\"reuse\":%d,\"stored\":%d,\"same\":%d,%s}\n",
                         detect_index,
                         (double)alpha,
                         (int)info.reuse_bvh_topology(),
                         (int)grown,
                         (int)same,
                         queries.c_str());
            std::fflush(log);
            ++detect_index;
        }
        return host_counts;
    };

    // Judges the stored pairs into the candidate lists (in the query formats)
    // and downloads their counts and the escape flag. One judge launch per
    // list (two for the cover EE list of a trajectory detect) writes flags and
    // per-tile counts, then one scan and one scatter launch compact all five
    // lists.
    auto judge_stored = [&]
    {
        std::array<ThisBVH::QueryBuffer*, JudgedLists> stored = {
            &stored_AllP_CodimP_pairs,
            &stored_CodimP_AllE_pairs,
            &stored_AllE_AllE_pairs,
            &stored_AllP_AllT_pairs,
            &stored_AllP_CoverT_pairs};
        std::array<ThisBVH::QueryBuffer*, JudgedLists> out = {
            &candidate_AllP_CodimP_pairs,
            &candidate_CodimP_AllE_pairs,
            &candidate_AllE_AllE_pairs,
            &candidate_AllP_AllT_pairs,
            &candidate_AllP_CoverT_pairs};
        if(!cover_PT)  // the PcoverT list stays empty
            stored[4] = nullptr;

        InfoStacklessBVHSimplexTrajectoryFilter_judged_lists lists;
        SizeT                                                pairs = 0;
        int                                                  tiles = 0;
        for(int l = 0; l < JudgedLists; ++l)
        {
            int n         = stored[l] ? static_cast<int>(stored[l]->size()) : 0;
            lists.list[l] = {stored[l] ? stored[l]->m_pairs.data() : nullptr,
                             nullptr,
                             out[l]->m_pairs.data(),
                             out[l]->m_cpNum.data(),
                             n,
                             tiles};
            pairs += n;
            tiles += (n + JudgeTile - 1) / JudgeTile;
        }
        stored_flags.resize_discard(pairs);
        judge_tile_counts.resize_discard(tiles);
        judge_tile_offsets.resize_discard(tiles);

        // Judge list l with leaf_pred; the stored pairs already passed the
        // position-free checks (see the judge kernel).
        SizeT flag_offset = 0;
        auto  judge       = [&](int                          l,
                         cuda_tool::CBufferView<AABB> query_aabbs,
                         cuda_tool::CBufferView<AABB> leaf_aabbs,
                         auto                         leaf_pred)
        {
            auto& L = lists.list[l];
            L.flags = stored_flags.data() + flag_offset;
            flag_offset += L.n;
            if(L.n == 0)
                return;
            int list_tiles = (L.n + JudgeTile - 1) / JudgeTile;
            bvh_pair_judge::judge_stored(
                stored[l]->view(),
                query_aabbs,
                leaf_aabbs,
                leaf_pred,
                stored_flags.view(L.flags - stored_flags.data(), L.n),
                judge_tile_counts.view(L.first_tile, list_tiles),
                L.n);
        };

        auto ee_pred               = alle_alle_pred;
        auto pt_pred               = allp_allt_pred;
        ee_pred.skip_position_free = true;
        pt_pred.skip_position_free = true;
        judge(0, point_aabbs, codim_point_aabbs, allp_codimp_pred);
        judge(1, codim_point_aabbs, edge_aabbs, codimp_alle_pred);
        if(cover_EE)
        {
            // the overlapping pairs grouped by call site (see judge_cover_ee_kernel),
            // screened first in a trajectory detect (see judge_screen_ee_kernel)
            auto& E = cover_edges;
            auto& L = lists.list[2];
            L.flags = stored_flags.data() + flag_offset;
            flag_offset += L.n;
            if(L.n > 0)
            {
                const int list_tiles = (L.n + JudgeTile - 1) / JudgeTile;
                const auto cover = InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred{
                    E.ids.view(0, E.count), E.is_query.view(), ee_pred};
                const auto flags =
                    stored_flags.view(L.flags - stored_flags.data(), L.n);
                const auto counts = judge_tile_counts.view(L.first_tile, list_tiles);
                InfoStacklessBVHSimplexTrajectoryFilter_judge_ee_queue queue{
                    nullptr, nullptr, judge_ee_queue_counts.data(), L.n};
                if(ee_pred.alpha != 0.0 && ee_pred.f32_positions.size())
                {
                    judge_ee_queue.resize_discard(2 * static_cast<SizeT>(L.n));
                    queue.second = judge_ee_queue.data();
                    queue.pairs  = judge_ee_queue.data() + L.n;
                }
                bvh_pair_judge::judge_ee(
                    stored[2]->view(), E.aabbs, edge_aabbs, cover, flags, counts, queue, judge_sm_count);
            }
        }
        else
        {
            judge(2, edge_aabbs, edge_aabbs, ee_pred);
        }
        if(cover_PT)
        {
            auto& P = cover_points;
            auto& T = cover_triangles;
            judge(3,
                  P.aabbs.view(0, P.count),
                  triangle_aabbs,
                  InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred{
                      P.ids.view(0, P.count), {}, pt_pred});
            judge(4,
                  P.aabbs.view(P.count),
                  T.aabbs,
                  InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred{
                      P.ids.view(P.count), T.ids.view(0, T.count), pt_pred});
        }
        else
        {
            judge(3, point_aabbs, triangle_aabbs, pt_pred);
            judge(4, {}, {}, pt_pred);  // empty
        }

        judged_counts.resize(8);
        bvh_pair_judge::compact(lists,
                                judge_tile_counts.view(),
                                judge_tile_offsets.view(),
                                stored_escaped.view(),
                                cuda_tool::BufferView<IndexT>{judged_counts.device_data(), 8},
                                judge_ee_queue_counts.data(),
                                tiles,
                                judge_sm_count);
        judged_counts.enqueue_copy(nullptr);
        if(!judged)
            CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&judged, cudaEventDisableTiming));
        CUDA_TOOL_CHECK(cudaEventRecord(judged, nullptr));

        // The cover remaps (see remap below) follow before the read-back.
        InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_segments remaps;
        auto remap = [&](ThisBVH::QueryBuffer& in,
                         const IndexT*         query_ids,
                         const IndexT*         leaf_ids,
                         bool                  ordered,
                         ThisBVH::QueryBuffer& out,
                         const int*            offset)
        {
            remaps.segment[remaps.segments++] = {in.m_pairs.data(),
                                                 query_ids,
                                                 leaf_ids,
                                                 ordered,
                                                 out.m_pairs.data(),
                                                 in.m_cpNum.data(),
                                                 offset};
        };
        if(cover_EE)
            remap(candidate_AllE_AllE_pairs, cover_edges.ids.data(), nullptr, true, candidate_AllE_AllE_pairs, nullptr);
        if(cover_PT)
        {
            auto& PT = candidate_AllP_AllT_pairs;
            remap(PT, cover_points.ids.data(), nullptr, false, PT, nullptr);
            remap(candidate_AllP_CoverT_pairs,
                  cover_points.ids.data() + cover_points.count,
                  cover_triangles.ids.data(),
                  false,
                  PT,
                  PT.m_cpNum.data());
        }
        if(remaps.segments > 0)
            InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_judged_kernel<<<judge_sm_count, 256>>>(
                remaps);

        if(!single_ef)
            info.before_read_back();
        CUDA_TOOL_CHECK(cudaEventSynchronize(judged));
        std::array<IndexT, 8> host_counts{};
        std::copy(judged_counts.data(), judged_counts.data() + 8, host_counts.begin());
        return host_counts;
    };

    std::array<IndexT, 8> host_counts{};
    if(!stored_enabled || direct_queries)
    {
        host_counts = run_queries(false);
    }
    else
    {
        // Judge the stored pairs right away; a box outside its grown copy
        // (flagged in the same download) discards that result.
        if(!refresh)
        {
            // (fused_prefix: the prefix kernel checked the boxes already)
            auto contained = [&](cuda_tool::CBufferView<AABB> aabbs,
                                 cuda_tool::CBufferView<AABB> grown,
                                 const CoverSplit*            split)
            {
                int n = static_cast<int>(aabbs.size());
                if(n == 0)
                    return;
                auto k = InfoStacklessBVHSimplexTrajectoryFilter_contained_kernel;
                k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                    aabbs,
                    grown,
                    split ? split->is_query.view() : cuda_tool::CBufferView<IndexT>{},
                    cover_shrink,
                    other_shrink,
                    stored_escaped.view(),
                    n);
            };
            if(!fused_prefix)
            {
                cuda_tool::BufferLaunch().fill(stored_escaped.view(), 0);
                contained(codim_point_aabbs, grown_codim_point_aabbs, nullptr);
                contained(point_aabbs, grown_point_aabbs, cover_PT ? &cover_points : nullptr);
                contained(edge_aabbs, grown_edge_aabbs, cover_EE ? &cover_edges : nullptr);
                contained(triangle_aabbs, grown_triangle_aabbs, cover_PT ? &cover_triangles : nullptr);
            }
            host_counts = judge_stored();
            refresh     = host_counts[5] != 0;
            frame_cover_left |= host_counts[6] != 0;
            frame_other_left |= host_counts[7] != 0;
            if(refresh)  // raise the levels that this detect left
            {
                stored_cover_thick |= host_counts[6] != 0;
                stored_other_uniform |= host_counts[7] != 0;
            }
        }
        else if(!info.reuse_bvh_topology())  // a new frame: the last one decides
        {
            stored_cover_thick   = frame_cover_left;
            stored_other_uniform = frame_other_left;
            frame_cover_left     = false;
            frame_other_left     = false;
        }
        if(refresh)
        {
            auto stored_skin = [&](Float over_d_hat)  // NaN: no skin
            {
                Float s = over_d_hat * info.d_hat();
                return s > 0 ? static_cast<float>(s) : 0.0f;
            };
            const float uniform = stored_skin(StoredSkinOverDHat);
            const float cover =
                stored_skin(stored_cover_thick ? StoredThickSkinOverDHat : StoredSkinOverDHat);
            const float other =
                stored_skin(stored_other_uniform ? StoredSkinOverDHat : StoredThinSkinOverDHat);
            // A gathered cover list grows each box as its primitive's own box.
            auto grow = [&](cuda_tool::CBufferView<AABB>   aabbs,
                            cuda_tool::DeviceBuffer<AABB>& grown,
                            const CoverSplit*              split,
                            bool                           gathered)
            {
                int n = static_cast<int>(aabbs.size());
                grown.resize_discard(n);
                if(n == 0)
                    return;
                cuda_tool::CBufferView<IndexT> ids, is_query;
                if(split)
                {
                    is_query = split->is_query.view();
                    if(gathered)
                        ids = split->ids.view(0, n);
                }
                auto k = InfoStacklessBVHSimplexTrajectoryFilter_grow_aabbs_kernel;
                k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                    aabbs,
                    ids,
                    is_query,
                    split ? cover : uniform,
                    split ? other : uniform,
                    grown.view(),
                    n);
            };
            const CoverSplit* E = cover_EE ? &cover_edges : nullptr;
            const CoverSplit* P = cover_PT ? &cover_points : nullptr;
            const CoverSplit* T = cover_PT ? &cover_triangles : nullptr;
            grow(codim_point_aabbs, grown_codim_point_aabbs, nullptr, false);
            grow(point_aabbs, grown_point_aabbs, P, false);
            grow(edge_aabbs, grown_edge_aabbs, E, false);
            grow(triangle_aabbs, grown_triangle_aabbs, T, false);
            if(cover_EE)
                grow(cover_edges.aabbs, cover_edges.grown_aabbs, E, true);
            if(cover_PT)
            {
                grow(cover_points.aabbs, cover_points.grown_aabbs, P, true);
                grow(cover_triangles.aabbs, cover_triangles.grown_aabbs, T, true);
            }

            auto stored_counts = run_queries(true);
            stored_valid       = true;
            stored_sizes       = sizes;

            // Every judged list fits its stored pairs (PT: plus the cover-tree ones).
            auto fit = [](ThisBVH::QueryBuffer& out, SizeT need)
            {
                if(out.m_pairs.size() < need)
                    out.reserve(static_cast<SizeT>(need * 1.2) + 1);
            };
            fit(candidate_AllP_CodimP_pairs, stored_counts[0]);
            fit(candidate_CodimP_AllE_pairs, stored_counts[1]);
            fit(candidate_AllE_AllE_pairs, stored_counts[2]);
            fit(candidate_AllP_AllT_pairs, stored_counts[3] + stored_counts[4]);
            fit(candidate_AllP_CoverT_pairs, stored_counts[4]);
            host_counts = judge_stored();
        }
        // Publish the counts; the lists were sized for every stored pair.
        bool grew =
            lbvh_CodimP.prepare_query_result(candidate_AllP_CodimP_pairs, host_counts[0])
            | lbvh_E.prepare_query_result(candidate_CodimP_AllE_pairs, host_counts[1])
            | lbvh_E.prepare_query_result(candidate_AllE_AllE_pairs, host_counts[2])
            | lbvh_T.prepare_query_result(candidate_AllP_AllT_pairs,
                                          host_counts[3] + host_counts[4])
            | lbvh_T_cover.prepare_query_result(candidate_AllP_CoverT_pairs,
                                                host_counts[4]);
        UIPC_ASSERT(!grew, "Judged stored pairs exceed their candidate lists");
        return;  // judge_stored() remapped the cover pairs
    }
    IndexT covert_count = host_counts[4];

    // Rewrite cover (compacted query, leaf) pairs into the original formats.
    auto remap = [](cuda_tool::CBufferView<Vector2i> query_pairs,
                    cuda_tool::CBufferView<IndexT>   query_ids,
                    cuda_tool::CBufferView<IndexT>   leaf_ids,
                    bool                             ordered,
                    cuda_tool::BufferView<Vector2i>  pairs)
    {
        int n = static_cast<int>(pairs.size());
        if(n == 0)
            return;
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_cover_remap_pairs_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            query_pairs, query_ids, leaf_ids, ordered, pairs, n);
    };
    if(cover_EE)
    {
        auto& pairs = candidate_AllE_AllE_pairs.m_pairs;
        auto  n     = static_cast<SizeT>(host_counts[2]);
        remap(pairs.cview(0, n),
              cover_edges.ids.view(0, cover_edges.count),
              {},
              true,
              pairs.view(0, n));
    }
    if(cover_PT)
    {
        auto& pairs = candidate_AllP_AllT_pairs.m_pairs;
        auto  n     = static_cast<SizeT>(host_counts[3]);
        auto  m     = static_cast<SizeT>(covert_count);
        auto& P     = cover_points;
        remap(pairs.cview(0, n), P.ids.view(0, P.count), {}, false, pairs.view(0, n));
        remap(candidate_AllP_CoverT_pairs.m_pairs.cview(0, m),
              P.ids.view(P.count),
              cover_triangles.ids.view(0, cover_triangles.count),
              false,
              pairs.view(n, m));
    }
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
