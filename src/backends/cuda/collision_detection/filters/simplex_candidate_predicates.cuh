#pragma once
#include <collision_detection/info_stackless_bvh.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <utils/simplex_contact_mask_utils.h>
#include <utils/primitive_d_hat.h>

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

    __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto        i  = info.i;
        auto        j  = info.j;
        const auto& E0 = Es(i);
        const auto& E1 = Es(j);

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
            // degenerate handling in filter_active
            Float D2;
            Vector4i flag = distance::edge_edge_distance_flag(E0_0, E0_1, E1_0, E1_1);
            distance::edge_edge_distance2(flag, E0_0, E0_1, E1_0, E1_1, D2);
            return D2 < expand * expand;
        }

        Vector3 dE0_0 = alpha * dxs(E0[0]);
        Vector3 dE0_1 = alpha * dxs(E0[1]);
        Vector3 dE1_0 = alpha * dxs(E1[0]);
        Vector3 dE1_1 = alpha * dxs(E1[1]);

        if(!distance::edge_edge_ccd_broadphase(
               E0_0, E0_1, E1_0, E1_1, dE0_0, dE0_1, dE1_0, dE1_1, expand))
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

    __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
    {
        auto i = info.i;
        auto j = info.j;
        auto V = Vs(i);
        auto F = Fs(j);

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
            // DCD detect pass: exact distance test (see PP pred)
            Float D2;
            Vector4i flag = distance::point_triangle_distance_flag(P, F0, F1, F2);
            distance::point_triangle_distance2(flag, P, F0, F1, F2, D2);
            return D2 < expand * expand;
        }

        Vector3 dP = alpha * dxs(V);

        Vector3 dF0 = alpha * dxs(F[0]);
        Vector3 dF1 = alpha * dxs(F[1]);
        Vector3 dF2 = alpha * dxs(F[2]);

        if(!distance::point_triangle_ccd_broadphase(P, F0, F1, F2, dP, dF0, dF1, dF2, expand))
            return false;

        return true;
    }
};

}  // namespace uipc::backend::cuda::bvh_simplex_detail
