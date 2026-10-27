#include <collision_detection/filters/single_ef_bvh_simplex_trajectory_filter.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(SingleEFBVHSimplexTrajectoryFilter);

void SingleEFBVHSimplexTrajectoryFilter::do_build(BuildInfo&)
{
    auto&       config = world().scene().config();
    auto        method = config.find<std::string>("collision_detection/method");
    const auto& selected = method->view()[0];
    if(selected != "single_ef_bvh" && selected != "single_ef_optix")
    {
        throw SimSystemException("Single EF BVH unused");
    }

    m_pipeline.single_ef.set_backend(selected == "single_ef_optix" ?
                                         SingleEFBVHQuery::Backend::Optix :
                                         SingleEFBVHQuery::Backend::Cuda);

    m_pipeline.init();
}

void SingleEFBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_pipeline.detect(info, BVHSimplexPipeline::CandidateMethod::SingleEF);
}

void SingleEFBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_pipeline.filter_active(info);
}

void SingleEFBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_pipeline.filter_toi(info);
}

cuda_tool::CBufferView<Vector2i> SingleEFBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_pipeline.candidate_AllP_AllT_pairs.view();
}

cuda_tool::CBufferView<Vector2i> SingleEFBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_pipeline.candidate_AllE_AllE_pairs.view();
}

cuda_tool::CBufferView<Float> SingleEFBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_pipeline.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_pipeline.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_pipeline.candidate_AllP_AllT_pairs.size();
    return m_pipeline.tois.view(pp_size + pe_size, pt_size);
}

cuda_tool::CBufferView<Float> SingleEFBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_pipeline.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_pipeline.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_pipeline.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_pipeline.candidate_AllE_AllE_pairs.size();
    return m_pipeline.tois.view(pp_size + pe_size + pt_size, ee_size);
}

}  // namespace uipc::backend::cuda
