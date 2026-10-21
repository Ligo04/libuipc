#pragma once
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/filters/single_ef_bvh_query.h>
#include <memory>

namespace uipc::backend::cuda
{
class BVHSimplexPipeline
{
  public:
    using DetectInfo       = SimplexTrajectoryFilter::DetectInfo;
    using FilterActiveInfo = SimplexTrajectoryFilter::FilterActiveInfo;
    using FilterTOIInfo    = SimplexTrajectoryFilter::FilterTOIInfo;
    enum class CandidateMethod
    {
        Baseline,
        SingleEF
    };
    void init()
    {
        query_counts.resize(4);
        selected_counts.resize(4);
    }
    void detect(DetectInfo& info, CandidateMethod method);
    void filter_active(FilterActiveInfo& info);
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

    SingleEFBVHQuery single_ef;

    using ThisBVH = InfoStacklessBVH;

    // CodimP count always less or equal to AllP count.
    ThisBVH              lbvh_CodimP;
    ThisBVH::QueryBuffer candidate_AllP_CodimP_pairs;

    // Used to detect CodimP-AllE, and AllE-AllE pairs.
    ThisBVH              lbvh_E;
    ThisBVH::QueryBuffer candidate_CodimP_AllE_pairs;
    ThisBVH::QueryBuffer candidate_AllE_AllE_pairs;

    // Used to detect AllP-AllT pairs.
    ThisBVH              lbvh_T;
    ThisBVH::QueryBuffer candidate_AllP_AllT_pairs;

    // Four query counts and four compacted PP/PE/PT/EE counts are each
    // downloaded in one contiguous transfer.
    cuda_tool::DeviceBuffer<IndexT> query_counts;
    cuda_tool::DeviceBuffer<IndexT> selected_counts;

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

  private:
    void audit_detect(DetectInfo& info, CandidateMethod method, bool used_single_ef);
    void                                audit_active();
    void                                audit_toi(FilterTOIInfo& info);
    std::unique_ptr<BVHSimplexPipeline> m_audit_shadow;
    U64                                 m_audit_sequence       = 0;
    Float                               m_audit_alpha          = 0.0;
    bool                                m_audit_used_single_ef = false;
    U64 m_audit_dcd_single = 0, m_audit_ccd_single = 0;
    U64 m_audit_dcd_fallback = 0, m_audit_ccd_fallback = 0;
};

}  // namespace uipc::backend::cuda
