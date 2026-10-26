#include <collision_detection/filters/bvh_simplex_pipeline.h>
#include <uipc/common/json.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace uipc::backend::cuda
{
namespace
{
    const std::string& audit_file()
    {
        static const std::string path = []
        {
            const char* value = std::getenv("UIPC_SINGLE_EF_AUDIT_FILE");
            return value ? std::string(value) : std::string();
        }();
        return path;
    }

    enum class KeyKind
    {
        CandidatePP,
        CandidatePE,
        CandidatePT,
        CandidateEE,
        PP,
        PE,
        PT,
        EE
    };

    template <int N>
    auto read_keys(cuda_tool::CBufferView<Vector<IndexT, N>> view, KeyKind kind)
    {
        std::vector<Vector<IndexT, N>> raw(view.size());
        if(!raw.empty())
            view.copy_to(raw.data());
        std::vector<std::array<IndexT, N>> keys(raw.size());
        for(SizeT i = 0; i < raw.size(); ++i)
        {
            auto& key = keys[i];
            for(int j = 0; j < N; ++j)
                key[j] = raw[i][j];
            if constexpr(N == 2)
            {
                if((kind == KeyKind::CandidateEE || kind == KeyKind::PP) && key[1] < key[0])
                    std::swap(key[0], key[1]);
            }
            if constexpr(N == 3)
            {
                if(key[2] < key[1])
                    std::swap(key[1], key[2]);
            }
            if constexpr(N == 4)
            {
                if(kind == KeyKind::PT)
                    std::sort(key.begin() + 1, key.end());
                else
                {
                    if(key[1] < key[0])
                        std::swap(key[0], key[1]);
                    if(key[3] < key[2])
                        std::swap(key[2], key[3]);
                    if(std::array<IndexT, 2>{key[2], key[3]}
                       < std::array<IndexT, 2>{key[0], key[1]})
                    {
                        std::swap(key[0], key[2]);
                        std::swap(key[1], key[3]);
                    }
                }
            }
        }
        std::sort(keys.begin(), keys.end());
        return keys;
    }

    template <int N>
    Json compare(cuda_tool::CBufferView<Vector<IndexT, N>> actual,
                 cuda_tool::CBufferView<Vector<IndexT, N>> baseline,
                 KeyKind                                   kind)
    {
        const auto lhs = read_keys<N>(actual, kind);
        const auto rhs = read_keys<N>(baseline, kind);
        using Key      = std::array<IndexT, N>;
        std::vector<Key> missing, extra;
        std::set_difference(rhs.begin(), rhs.end(), lhs.begin(), lhs.end(), std::back_inserter(missing));
        std::set_difference(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::back_inserter(extra));
        auto duplicates = [](const auto& sorted)
        {
            SizeT count = 0;
            for(SizeT i = 1; i < sorted.size(); ++i)
                count += sorted[i] == sorted[i - 1];
            return count;
        };
        Json result{{"actual_count", lhs.size()},
                    {"baseline_count", rhs.size()},
                    {"actual_duplicates", duplicates(lhs)},
                    {"baseline_duplicates", duplicates(rhs)},
                    {"missing", missing.size()},
                    {"extra", extra.size()},
                    {"equal", lhs == rhs}};
        missing.resize(std::min<SizeT>(missing.size(), 8));
        extra.resize(std::min<SizeT>(extra.size(), 8));
        result["missing_sample"] = missing;
        result["extra_sample"]   = extra;
        return result;
    }

    constexpr Float toi_absolute_tolerance = 1.0e-12;
    constexpr Float toi_relative_tolerance = 1.0e-9;

    bool close_toi(Float lhs, Float rhs)
    {
        return std::isfinite(lhs) && std::isfinite(rhs)
               && std::abs(lhs - rhs)
                      <= toi_absolute_tolerance
                             + toi_relative_tolerance
                                   * std::max(std::abs(lhs), std::abs(rhs));
    }

    Json finite_value(Float value)
    {
        return std::isfinite(value) ? Json(value) : Json(nullptr);
    }

    struct PairTOI
    {
        std::array<IndexT, 2> key;
        Float                 value;
    };

    auto read_pair_tois(cuda_tool::CBufferView<Vector2i> pairs,
                        cuda_tool::CBufferView<Float>    tois,
                        KeyKind                          kind)
    {
        if(pairs.size() != tois.size())
            throw std::runtime_error("Single EF TOI audit candidate/value sizes differ");
        std::vector<Vector2i> host_pairs(pairs.size());
        std::vector<Float>    host_tois(tois.size());
        if(!host_pairs.empty())
        {
            pairs.copy_to(host_pairs.data());
            tois.copy_to(host_tois.data());
        }
        std::vector<PairTOI> result(pairs.size());
        for(SizeT i = 0; i < result.size(); ++i)
        {
            auto& entry = result[i];
            entry.key   = {host_pairs[i][0], host_pairs[i][1]};
            if(kind == KeyKind::CandidateEE && entry.key[1] < entry.key[0])
                std::swap(entry.key[0], entry.key[1]);
            entry.value = host_tois[i];
        }
        // Match duplicate keys as value multisets without sorting NaNs.
        std::sort(result.begin(),
                  result.end(),
                  [](const PairTOI& lhs, const PairTOI& rhs)
                  {
                      if(lhs.key != rhs.key)
                          return lhs.key < rhs.key;
                      const bool a = std::isfinite(lhs.value),
                                 b = std::isfinite(rhs.value);
                      if(a != b)
                          return a;
                      return a && lhs.value < rhs.value;
                  });
        return result;
    }

    Json compare_pair_tois(cuda_tool::CBufferView<Vector2i> actual_pairs,
                           cuda_tool::CBufferView<Float>    actual_tois,
                           cuda_tool::CBufferView<Vector2i> baseline_pairs,
                           cuda_tool::CBufferView<Float>    baseline_tois,
                           KeyKind                          kind)
    {
        const auto actual = read_pair_tois(actual_pairs, actual_tois, kind);
        const auto baseline = read_pair_tois(baseline_pairs, baseline_tois, kind);
        auto summarize = [](const auto& entries)
        {
            SizeT duplicates = 0, nonfinite = 0;
            Json  minimum = nullptr;
            for(SizeT i = 0; i < entries.size(); ++i)
            {
                duplicates += i > 0 && entries[i].key == entries[i - 1].key;
                if(!std::isfinite(entries[i].value))
                    ++nonfinite;
                else if(minimum.is_null()
                        || entries[i].value < minimum.template get<Float>())
                    minimum = entries[i].value;
            }
            return Json{{"duplicates", duplicates}, {"nonfinite", nonfinite}, {"min", minimum}};
        };
        const auto lhs = summarize(actual), rhs = summarize(baseline);
        SizeT i = 0, j = 0, missing = 0, extra = 0, mismatches = 0, compared = 0;
        Float max_abs = 0.0, max_rel = 0.0;
        Json  samples = Json::array();
        while(i < actual.size() && j < baseline.size())
        {
            if(actual[i].key < baseline[j].key)
            {
                ++extra;
                ++i;
                continue;
            }
            if(baseline[j].key < actual[i].key)
            {
                ++missing;
                ++j;
                continue;
            }
            const auto a = actual[i].value, b = baseline[j].value;
            ++compared;
            if(std::isfinite(a) && std::isfinite(b))
            {
                const auto absolute = std::abs(a - b);
                const auto scale    = std::max(std::abs(a), std::abs(b));
                max_abs             = std::max(max_abs, absolute);
                max_rel = std::max(max_rel, scale > 0.0 ? absolute / scale : 0.0);
            }
            if(!close_toi(a, b))
            {
                ++mismatches;
                if(samples.size() < 8)
                    samples.push_back(Json{{"key", actual[i].key},
                                           {"actual", finite_value(a)},
                                           {"baseline", finite_value(b)}});
            }
            ++i;
            ++j;
        }
        extra += actual.size() - i;
        missing += baseline.size() - j;
        const bool equal = missing == 0 && extra == 0 && mismatches == 0
                           && lhs["nonfinite"] == 0 && rhs["nonfinite"] == 0;
        return Json{{"actual_count", actual.size()},
                    {"baseline_count", baseline.size()},
                    {"actual_duplicates", lhs["duplicates"]},
                    {"baseline_duplicates", rhs["duplicates"]},
                    {"missing", missing},
                    {"extra", extra},
                    {"compared_pairs", compared},
                    {"value_mismatches", mismatches},
                    {"nonfinite_actual", lhs["nonfinite"]},
                    {"nonfinite_baseline", rhs["nonfinite"]},
                    {"actual_min", lhs["min"]},
                    {"baseline_min", rhs["min"]},
                    {"max_abs_diff", max_abs},
                    {"max_rel_diff", max_rel},
                    {"mismatch_sample", samples},
                    {"equal", equal}};
    }

    void write_record(const Json& record)
    {
        static std::mutex mutex;
        std::lock_guard   lock(mutex);
        std::ofstream     file(audit_file(), std::ios::app);
        if(!file)
            throw std::runtime_error("Cannot open UIPC_SINGLE_EF_AUDIT_FILE");
        file << record.dump() << '\n';
        file.flush();
        if(!file)
            throw std::runtime_error("Cannot write UIPC_SINGLE_EF_AUDIT_FILE");
    }
}  // namespace

void BVHSimplexPipeline::audit_detect(DetectInfo& info, CandidateMethod method, bool used_single_ef)
{
    if(method != CandidateMethod::SingleEF || audit_file().empty())
        return;
    if(!m_audit_shadow)
    {
        m_audit_shadow = std::make_unique<BVHSimplexPipeline>();
        m_audit_shadow->init();
    }
    static std::atomic<U64> sequence{0};
    m_audit_sequence       = ++sequence;
    m_audit_alpha          = info.alpha();
    m_audit_used_single_ef = used_single_ef;
    if(used_single_ef)
    {
        if(m_audit_alpha == 0.0)
            ++m_audit_dcd_single;
        else
            ++m_audit_ccd_single;
    }
    else if(m_audit_alpha != 0.0)
        ++m_audit_ccd_fallback;
    else
        ++m_audit_dcd_fallback;

    m_audit_shadow->detect(info, CandidateMethod::Baseline);
    Json pp = compare<2>(candidate_AllP_CodimP_pairs.view(),
                         m_audit_shadow->candidate_AllP_CodimP_pairs.view(),
                         KeyKind::CandidatePP);
    Json pe = compare<2>(candidate_CodimP_AllE_pairs.view(),
                         m_audit_shadow->candidate_CodimP_AllE_pairs.view(),
                         KeyKind::CandidatePE);
    Json pt                    = compare<2>(candidate_AllP_AllT_pairs.view(),
                         m_audit_shadow->candidate_AllP_AllT_pairs.view(),
                         KeyKind::CandidatePT);
    Json ee                    = compare<2>(candidate_AllE_AllE_pairs.view(),
                         m_audit_shadow->candidate_AllE_AllE_pairs.view(),
                         KeyKind::CandidateEE);
    pt["actual_writable_size"] = candidate_AllP_AllT_pairs.m_pairs.size();
    pt["baseline_writable_size"] =
        m_audit_shadow->candidate_AllP_AllT_pairs.m_pairs.size();
    ee["actual_writable_size"] = candidate_AllE_AllE_pairs.m_pairs.size();
    ee["baseline_writable_size"] =
        m_audit_shadow->candidate_AllE_AllE_pairs.m_pairs.size();
    bool equal = pp["equal"].get<bool>() && pe["equal"].get<bool>()
                 && pt["equal"].get<bool>() && ee["equal"].get<bool>();
    write_record(Json{{"detect_sequence", m_audit_sequence},
                      {"phase", "candidates"},
                      {"comparison", "canonical_multiset"},
                      {"alpha", m_audit_alpha},
                      {"mode", m_audit_alpha == 0.0 ? "DCD" : "CCD"},
                      {"used_single_ef", used_single_ef},
                      {"codim_vertices", info.codim_vertices().size()},
                      {"isolated_point_proxies", used_single_ef ? single_ef.isolated_point_count() : 0},
                      {"standalone_edge_proxies", used_single_ef ? single_ef.standalone_edge_count() : 0},
                      {"equal", equal},
                      {"dcd_single_calls", m_audit_dcd_single},
                      {"dcd_fallback_calls", m_audit_dcd_fallback},
                      {"ccd_single_calls", m_audit_ccd_single},
                      {"ccd_fallback_calls", m_audit_ccd_fallback},
                      {"toi_compared", false},
                      {"PP", pp},
                      {"PE", pe},
                      {"PT", pt},
                      {"EE", ee}});
    if(!equal)
        throw std::runtime_error("Single EF candidate audit mismatch; see audit JSONL");
}

void BVHSimplexPipeline::audit_active()
{
    Json pp = compare<2>(PPs.view(), m_audit_shadow->PPs.view(), KeyKind::PP);
    Json pe = compare<3>(PEs.view(), m_audit_shadow->PEs.view(), KeyKind::PE);
    Json pt = compare<4>(PTs.view(), m_audit_shadow->PTs.view(), KeyKind::PT);
    Json ee = compare<4>(EEs.view(), m_audit_shadow->EEs.view(), KeyKind::EE);
    bool equal = pp["equal"].get<bool>() && pe["equal"].get<bool>()
                 && pt["equal"].get<bool>() && ee["equal"].get<bool>();
    write_record(Json{{"detect_sequence", m_audit_sequence},
                      {"phase", "active"},
                      {"comparison", "canonical_multiset"},
                      {"alpha", m_audit_alpha},
                      {"used_single_ef", m_audit_used_single_ef},
                      {"equal", equal},
                      {"toi_compared", false},
                      {"PP", pp},
                      {"PE", pe},
                      {"PT", pt},
                      {"EE", ee}});
    if(!equal)
        throw std::runtime_error("Single EF active audit mismatch; see audit JSONL");
}

void BVHSimplexPipeline::audit_toi(FilterTOIInfo& info)
{
    Float actual_min = 0.0, baseline_min = 0.0;
    info.toi().copy_to(&actual_min);
    auto restore_actual_min = [&]
    {
        info.toi().copy_from(&actual_min);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
    };
    try
    {
        m_audit_shadow->filter_toi(info);
        info.toi().copy_to(&baseline_min);
    }
    catch(...)
    {
        restore_actual_min();
        throw;
    }
    // The shadow must never publish its reduction into the real simulation.
    restore_actual_min();

    Json pp = compare_pair_tois(
        candidate_AllP_CodimP_pairs.view(),
        tois.view(0, candidate_AllP_CodimP_pairs.size()),
        m_audit_shadow->candidate_AllP_CodimP_pairs.view(),
        m_audit_shadow->tois.view(0, m_audit_shadow->candidate_AllP_CodimP_pairs.size()),
        KeyKind::CandidatePP);
    Json pe = compare_pair_tois(
        candidate_CodimP_AllE_pairs.view(),
        tois.view(candidate_AllP_CodimP_pairs.size(), candidate_CodimP_AllE_pairs.size()),
        m_audit_shadow->candidate_CodimP_AllE_pairs.view(),
        m_audit_shadow->tois.view(m_audit_shadow->candidate_AllP_CodimP_pairs.size(),
                                  m_audit_shadow->candidate_CodimP_AllE_pairs.size()),
        KeyKind::CandidatePE);
    const auto actual_offset =
        candidate_AllP_CodimP_pairs.size() + candidate_CodimP_AllE_pairs.size();
    const auto shadow_offset = m_audit_shadow->candidate_AllP_CodimP_pairs.size()
                               + m_audit_shadow->candidate_CodimP_AllE_pairs.size();
    Json pt = compare_pair_tois(
        candidate_AllP_AllT_pairs.view(),
        tois.view(actual_offset, candidate_AllP_AllT_pairs.size()),
        m_audit_shadow->candidate_AllP_AllT_pairs.view(),
        m_audit_shadow->tois.view(shadow_offset,
                                  m_audit_shadow->candidate_AllP_AllT_pairs.size()),
        KeyKind::CandidatePT);
    Json ee = compare_pair_tois(
        candidate_AllE_AllE_pairs.view(),
        tois.view(actual_offset + candidate_AllP_AllT_pairs.size(),
                  candidate_AllE_AllE_pairs.size()),
        m_audit_shadow->candidate_AllE_AllE_pairs.view(),
        m_audit_shadow->tois.view(
            shadow_offset + m_audit_shadow->candidate_AllP_AllT_pairs.size(),
            m_audit_shadow->candidate_AllE_AllE_pairs.size()),
        KeyKind::CandidateEE);
    const bool minimum_equal = close_toi(actual_min, baseline_min);
    const bool equal =
        minimum_equal && pp["equal"].get<bool>() && pe["equal"].get<bool>()
        && pt["equal"].get<bool>() && ee["equal"].get<bool>();
    write_record(Json{{"detect_sequence", m_audit_sequence},
                      {"phase", "toi"},
                      {"comparison", "canonical_pair_toi_multiset"},
                      {"alpha", info.alpha()},
                      {"used_single_ef", m_audit_used_single_ef},
                      {"equal", equal},
                      {"toi_compared", true},
                      {"abs_tolerance", toi_absolute_tolerance},
                      {"rel_tolerance", toi_relative_tolerance},
                      {"actual_min", finite_value(actual_min)},
                      {"baseline_min", finite_value(baseline_min)},
                      {"minimum_equal", minimum_equal},
                      {"minimum_abs_diff",
                       std::isfinite(actual_min) && std::isfinite(baseline_min) ?
                           Json(std::abs(actual_min - baseline_min)) :
                           Json(nullptr)},
                      {"PP", pp},
                      {"PE", pe},
                      {"PT", pt},
                      {"EE", ee}});
    if(!equal)
        throw std::runtime_error("Single EF pairwise TOI audit mismatch; see audit JSONL");
}
}  // namespace uipc::backend::cuda
