#include <collision_detection/filters/single_ef_bvh_simplex_trajectory_filter.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(SingleEFBVHSimplexTrajectoryFilter);

void SingleEFBVHSimplexTrajectoryFilter::do_build(BuildInfo&)
{
    auto method = world().scene().config().find<std::string>("collision_detection/method");
    if(method->view()[0] != "single_ef_bvh")
        throw SimSystemException("Single EF BVH unused");
    m_impl.single_ef = true;
    m_impl.init();
}

void SingleEFBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
    if(m_impl.check_stored_pairs)
        m_impl.check_detect(info);
}

void SingleEFBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.launch_filter_active(info);
    CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    m_impl.finish_filter_active(info);
}

bool SingleEFBVHSimplexTrajectoryFilter::do_launch_filter_active(FilterActiveInfo& info)
{
    m_impl.launch_filter_active(info);
    return true;
}

void SingleEFBVHSimplexTrajectoryFilter::do_finish_filter_active(FilterActiveInfo& info)
{
    m_impl.finish_filter_active(info);
}

void SingleEFBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

cuda_tool::CBufferView<Vector2i> SingleEFBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

cuda_tool::CBufferView<Vector2i> SingleEFBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

cuda_tool::CBufferView<Float> SingleEFBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

cuda_tool::CBufferView<Float> SingleEFBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}

}  // namespace uipc::backend::cuda
