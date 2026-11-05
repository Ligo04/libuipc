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

#pragma once
#include <collision_detection/info_stackless_bvh.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <utils/simplex_contact_mask_utils.h>
#include <utils/primitive_d_hat.h>
#include <utils/distance/float_lower_bound.h>
#include <utils/distance/swept_lower_bound.h>
#include <utils/distance/swept_lower_bound_f32.h>
#include <utils/step_displacement.h>

namespace uipc::backend::cuda::bvh_simplex_detail
{
struct NodePredicate
{
    cuda_tool::CBufferView<IndexT> body_self_collision;
    cuda_tool::CDense2D<IndexT>    cmts;

    __device__ bool operator()(InfoStacklessBVH::NodePredInfo info) const
    {
        constexpr IndexT invalid  = static_cast<IndexT>(-1);
        auto             qbid     = info.query_bid;
        auto             qcid     = info.query_cid;
        bool             bid_cull = info.node_bid != invalid && qbid != invalid
                        && qbid == info.node_bid && !body_self_collision(qbid);
        bool cid_cull = info.node_cid != invalid && qcid != invalid
                        && !cmts(qcid, info.node_cid);
        return !(bid_cull || cid_cull);
    }
};

struct PointPointPredicate
{
    cuda_tool::CBufferView<IndexT>  Vs;
    cuda_tool::CBufferView<IndexT>  codimVs;
    cuda_tool::CBufferView<Vector3> Ps;
    cuda_tool::CBufferView<Vector3> dxs;
    cuda_tool::CBufferView<Float>   thicknesses;
    cuda_tool::CBufferView<IndexT>  dimensions;
    cuda_tool::CBufferView<IndexT>  contact_element_ids;
    cuda_tool::CDense2D<IndexT>     contact_mask_tabular;
    cuda_tool::CBufferView<IndexT>  subscene_element_ids;
    cuda_tool::CDense2D<IndexT>     subscene_mask_tabular;
    cuda_tool::CBufferView<IndexT>  body_self_collision;
    cuda_tool::CBufferView<Float>   d_hats;
    Float                           alpha;
    Float                           skin;  // trajectory tests only
    bool geometry = true;                  // false: position-free checks only

    __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto        i      = info.i;
        auto        j      = info.j;
        const auto& V      = Vs(i);
        const auto& codimV = codimVs(j);

        Vector2i cids = {contact_element_ids(V), contact_element_ids(codimV)};
        Vector2i scids = {subscene_element_ids(V), subscene_element_ids(codimV)};

        if(!allow_PP_contact(subscene_mask_tabular, scids))
            return false;
        if(!allow_PP_contact(contact_mask_tabular, cids))
            return false;

        bool V_is_codim = dimensions(V) <= 2;

        if(V_is_codim && V >= codimV)
            return false;

        if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
           && !body_self_collision(info.bid_i))
            return false;

        if(!geometry)
            return true;

        Vector3 P0 = Ps(V);
        Vector3 P1 = Ps(codimV);

        Float thickness = PP_thickness(thicknesses(V), thicknesses(codimV));
        Float d_hat     = PP_d_hat(d_hats(V), d_hats(codimV));

        Float expand = d_hat + thickness;

        if(alpha == 0.0)
        {
            // DCD detect pass: exact distance test. The conservative
            // ccd_broadphase degenerates to a per-axis box test at
            // alpha==0 and passes ~50x more candidates than the exact
            // activation filter downstream re-keeps anyway. Pairs with
            // D2 <= thickness^2 are kept so the thickness-violation
            // handling in filter_active still sees them.
            Float D2;
            distance::point_point_distance2(P0, P1, D2);
            return D2 < expand * expand;
        }

        Vector3 dP0 = alpha * dxs(V);
        Vector3 dP1 = alpha * dxs(codimV);

        if(!distance::point_point_ccd_broadphase(P0, P1, dP0, dP1, expand + skin))
            return false;

        return true;
    }
};

struct PointEdgePredicate
{
    cuda_tool::CBufferView<IndexT>   codimVs;
    cuda_tool::CBufferView<Vector2i> Es;
    cuda_tool::CBufferView<Vector3>  Ps;
    cuda_tool::CBufferView<Vector3>  dxs;
    cuda_tool::CBufferView<Float>    thicknesses;
    cuda_tool::CBufferView<IndexT>   contact_element_ids;
    cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
    cuda_tool::CBufferView<IndexT>   subscene_element_ids;
    cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
    cuda_tool::CBufferView<IndexT>   body_self_collision;
    cuda_tool::CBufferView<Float>    d_hats;
    Float                            alpha;
    Float                            skin;  // trajectory tests only
    bool geometry = true;                   // false: position-free checks only

    __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto        i      = info.i;
        auto        j      = info.j;
        const auto& codimV = codimVs(i);
        const auto& E      = Es(j);

        Vector3i cids = {contact_element_ids(codimV),
                         contact_element_ids(E[0]),
                         contact_element_ids(E[1])};

        Vector3i scids = {subscene_element_ids(codimV),
                          subscene_element_ids(E[0]),
                          subscene_element_ids(E[1])};

        if(!allow_PE_contact(subscene_mask_tabular, scids))
            return false;
        if(!allow_PE_contact(contact_mask_tabular, cids))
            return false;

        if(E[0] == codimV || E[1] == codimV)
            return false;

        if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
           && !body_self_collision(info.bid_i))
            return false;

        if(!geometry)
            return true;

        Vector3 E0 = Ps(E[0]);
        Vector3 E1 = Ps(E[1]);

        Vector3 P = Ps(codimV);

        Float thickness =
            PE_thickness(thicknesses(codimV), thicknesses(E[0]), thicknesses(E[1]));
        Float d_hat = PE_d_hat(d_hats(codimV), d_hats(E[0]), d_hats(E[1]));

        Float expand = d_hat + thickness;

        if(alpha == 0.0)
        {
            // DCD detect pass: exact distance test (see PP pred)
            Float    D2;
            Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);
            distance::point_edge_distance2(flag, P, E0, E1, D2);
            return D2 < expand * expand;
        }

        Vector3 dE0 = alpha * dxs(E[0]);
        Vector3 dE1 = alpha * dxs(E[1]);
        Vector3 dP  = alpha * dxs(codimV);

        if(!distance::point_edge_ccd_broadphase(P, E0, E1, dP, dE0, dE1, expand + skin))
            return false;

        return true;
    }
};

struct EdgeEdgePredicate
{
    cuda_tool::CBufferView<Vector2i> Es;
    cuda_tool::CBufferView<Vector3>  Ps;
    cuda_tool::CBufferView<Vector3>  dxs;
    cuda_tool::CBufferView<Float>    thicknesses;
    cuda_tool::CBufferView<IndexT>   contact_element_ids;
    cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
    cuda_tool::CBufferView<IndexT>   subscene_element_ids;
    cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
    cuda_tool::CBufferView<IndexT>   body_self_collision;
    cuda_tool::CBufferView<Float>    d_hats;
    Float                            alpha;
    Float                            skin;  // trajectory tests only
    bool geometry = true;                   // false: position-free checks only
    // Trajectory tests reject with FP32 inputs first (swept_lower_bound_f32.h);
    // empty views: FP64 inputs only.
    cuda_tool::CBufferView<float4> f32_positions;  // x, y, z, d_hat
    cuda_tool::CBufferView<float4> f32_steps;  // step displacement, thickness
    bool skip_position_free = false;  // stored pairs passed them at the refresh

    __device__ __forceinline__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto        i  = info.i;
        auto        j  = info.j;
        const auto& E0 = Es(i);
        const auto& E1 = Es(j);

        if(!skip_position_free)
        {
            Vector4i cids = {contact_element_ids(E0[0]),
                             contact_element_ids(E0[1]),
                             contact_element_ids(E1[0]),
                             contact_element_ids(E1[1])};

            Vector4i scids = {subscene_element_ids(E0[0]),
                              subscene_element_ids(E0[1]),
                              subscene_element_ids(E1[0]),
                              subscene_element_ids(E1[1])};

            if(!allow_EE_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_EE_contact(contact_mask_tabular, cids))
                return false;

            if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;
        }

        if(!geometry)
            return true;

        if(alpha != 0.0 && f32_positions.size() > 0
           && distance::edge_edge_swept_farther_than_f32(f32_positions(E0[0]),
                                                         f32_positions(E0[1]),
                                                         f32_positions(E1[0]),
                                                         f32_positions(E1[1]),
                                                         f32_steps(E0[0]),
                                                         f32_steps(E0[1]),
                                                         f32_steps(E1[0]),
                                                         f32_steps(E1[1]),
                                                         static_cast<float>(skin)))
            return false;

        Vector3 E0_0 = Ps(E0[0]);
        Vector3 E0_1 = Ps(E0[1]);

        Vector3 E1_0 = Ps(E1[0]);
        Vector3 E1_1 = Ps(E1[1]);

        Float thickness = EE_thickness(thicknesses(E0[0]),
                                       thicknesses(E0[1]),
                                       thicknesses(E1[0]),
                                       thicknesses(E1[1]));

        Float d_hat =
            EE_d_hat(d_hats(E0[0]), d_hats(E0[1]), d_hats(E1[0]), d_hats(E1[1]));

        Float expand = d_hat + thickness;

        if(alpha == 0.0)
        {
            // DCD detect pass: exact distance test (see PP pred); the
            // plain EE distance is a correct superset of the mollified
            // degenerate handling in filter_active. Most candidates are
            // far apart, so a conservative FP32 bound rejects them first.
            if(distance::edge_edge_farther_than(E0_0, E0_1, E1_0, E1_1, expand))
                return false;
            Float D2;
            Vector4i flag = distance::edge_edge_distance_flag(E0_0, E0_1, E1_0, E1_1);
            distance::edge_edge_distance2(flag, E0_0, E0_1, E1_0, E1_1, D2);
            return D2 < expand * expand;
        }

        Vector3 dE0_0 = step_displacement(alpha, dxs(E0[0]));
        Vector3 dE0_1 = step_displacement(alpha, dxs(E0[1]));
        Vector3 dE1_0 = step_displacement(alpha, dxs(E1[0]));
        Vector3 dE1_1 = step_displacement(alpha, dxs(E1[1]));

        // Most swept candidates stay far apart for the whole step; reject
        // them with a conservative FP32 bound before the FP64 box test.
        if(distance::edge_edge_swept_farther_than(
               E0_0, E0_1, E1_0, E1_1, dE0_0, dE0_1, dE1_0, dE1_1, expand + skin, thickness))
            return false;

        // step_displacement matches step_forward bit for bit, so the accepted
        // position (reused as the next iteration's DCD state) stays inside
        // this swept box.
        if(!distance::edge_edge_ccd_broadphase(
               E0_0, E0_1, E1_0, E1_1, dE0_0, dE0_1, dE1_0, dE1_1, expand + skin))
            return false;

        return true;
    }
};

struct PointTrianglePredicate
{
    cuda_tool::CBufferView<IndexT>   Vs;
    cuda_tool::CBufferView<Vector3i> Fs;
    cuda_tool::CBufferView<Vector3>  Ps;
    cuda_tool::CBufferView<Vector3>  dxs;
    cuda_tool::CBufferView<Float>    thicknesses;
    cuda_tool::CBufferView<IndexT>   contact_element_ids;
    cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
    cuda_tool::CBufferView<IndexT>   subscene_element_ids;
    cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
    cuda_tool::CBufferView<IndexT>   body_self_collision;
    cuda_tool::CBufferView<Float>    d_hats;
    Float                            alpha;
    Float                            skin;  // trajectory tests only
    bool geometry = true;                   // false: position-free checks only
    // See the AllE-AllE pred.
    cuda_tool::CBufferView<float4> f32_positions;
    cuda_tool::CBufferView<float4> f32_steps;
    bool                           skip_position_free = false;

    __device__ __forceinline__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto i = info.i;
        auto j = info.j;
        auto V = Vs(i);
        auto F = Fs(j);

        if(!skip_position_free)
        {
            Vector4i cids = {contact_element_ids(V),
                             contact_element_ids(F[0]),
                             contact_element_ids(F[1]),
                             contact_element_ids(F[2])};

            Vector4i scids = {subscene_element_ids(V),
                              subscene_element_ids(F[0]),
                              subscene_element_ids(F[1]),
                              subscene_element_ids(F[2])};

            if(!allow_PT_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PT_contact(contact_mask_tabular, cids))
                return false;

            if(F[0] == V || F[1] == V || F[2] == V)
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;
        }

        if(!geometry)
            return true;

        if(alpha != 0.0 && f32_positions.size() > 0
           && distance::point_triangle_swept_farther_than_f32(f32_positions(V),
                                                              f32_positions(F[0]),
                                                              f32_positions(F[1]),
                                                              f32_positions(F[2]),
                                                              f32_steps(V),
                                                              f32_steps(F[0]),
                                                              f32_steps(F[1]),
                                                              f32_steps(F[2]),
                                                              static_cast<float>(skin)))
            return false;

        Vector3 P = Ps(V);

        Vector3 F0 = Ps(F[0]);
        Vector3 F1 = Ps(F[1]);
        Vector3 F2 = Ps(F[2]);

        Float thickness = PT_thickness(
            thicknesses(V), thicknesses(F[0]), thicknesses(F[1]), thicknesses(F[2]));

        Float d_hat = PT_d_hat(d_hats(V), d_hats(F[0]), d_hats(F[1]), d_hats(F[2]));

        Float expand = d_hat + thickness;

        if(alpha == 0.0)
        {
            // DCD detect pass: exact distance test (see PP pred), after
            // the conservative FP32 rejection (see AllE-AllE pred)
            if(distance::point_triangle_farther_than(P, F0, F1, F2, expand))
                return false;
            Float D2;
            Vector4i flag = distance::point_triangle_distance_flag(P, F0, F1, F2);
            distance::point_triangle_distance2(flag, P, F0, F1, F2, D2);
            return D2 < expand * expand;
        }

        Vector3 dP = step_displacement(alpha, dxs(V));

        Vector3 dF0 = step_displacement(alpha, dxs(F[0]));
        Vector3 dF1 = step_displacement(alpha, dxs(F[1]));
        Vector3 dF2 = step_displacement(alpha, dxs(F[2]));

        // See the AllE-AllE pred: conservative swept FP32 rejection first.
        if(distance::point_triangle_swept_farther_than(
               P, F0, F1, F2, dP, dF0, dF1, dF2, expand + skin, thickness))
            return false;

        // See the AllE-AllE pred: exact end state via step_displacement.
        if(!distance::point_triangle_ccd_broadphase(P, F0, F1, F2, dP, dF0, dF1, dF2, expand + skin))
            return false;

        return true;
    }
};

/****************************************************
*               Contact-cover queries
****************************************************/

// A query edge of S_E meets leaf edge j. A leaf outside S_E is reached only
// from this side; a pair inside S_E is reached from both edges and kept
// from the smaller index. The pair is then judged exactly as the self-join
// judges it, as (min, max).
struct InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllE_AllE_pred
{
    cuda_tool::CBufferView<IndexT> query_edges;
    cuda_tool::CBufferView<IndexT> edge_is_query;
    EdgeEdgePredicate              pred;

    __device__ __forceinline__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        IndexT i = query_edges(info.i);
        IndexT j = info.j;
        if(!pred.skip_position_free && (i == j || (j < i && edge_is_query(j))))
            return false;
        if(i < j)
            return pred(InfoStacklessBVH::LeafPredInfo{
                i, j, info.bid_i, info.cid_i, info.bid_j, info.cid_j});
        return pred(InfoStacklessBVH::LeafPredInfo{
            j, i, info.bid_j, info.cid_j, info.bid_i, info.cid_i});
    }
};

// Points map back from their compacted query list and, in the cover tree,
// triangles from its compacted leaves.
struct InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred
{
    cuda_tool::CBufferView<IndexT> query_points;
    cuda_tool::CBufferView<IndexT> leaf_triangles;  // empty: leaves are triangle ids
    PointTrianglePredicate pred;

    __device__ __forceinline__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        info.i = query_points(info.i);
        if(leaf_triangles.size() > 0)
            info.j = leaf_triangles(info.j);
        return pred(info);
    }
};
}  // namespace uipc::backend::cuda::bvh_simplex_detail
