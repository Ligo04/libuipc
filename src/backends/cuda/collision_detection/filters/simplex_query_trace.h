#pragma once
#include <uipc/backend/macro.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <filesystem>
#include <memory>
#include <vector>

namespace uipc::backend::cuda::simplex_query_trace
{
// This format captures the production binder's shared predicate metadata.
// Independently bound standalone predicates are outside this trace contract.
struct Site
{
    SizeT frame = 0, newton_iteration = 0;
};
struct Matrix
{
    SizeT               rows = 0, cols = 0;
    std::vector<IndexT> values;
};
struct Metadata
{
    std::vector<Float>    d_hats, thicknesses;
    std::vector<IndexT>   v2bs, dimensions, contact_ids, subscene_ids;
    std::vector<IndexT>   body_self_collision, codim_vertices, surface_vertices;
    std::vector<Vector2i> edges;
    std::vector<Vector3i> triangles;
    Matrix                contact_mask, subscene_mask;
};
struct Request
{
    // Assigned by the codec; append does not use caller-supplied IDs.
    SizeT                           id = 0, epoch = 0;
    Site                            site;
    bool                            new_frame = false;
    Float                           alpha = 0, skin = 0;
    std::shared_ptr<const Metadata> metadata;
    std::vector<Vector3>            positions, displacements;
};
struct Summary
{
    SizeT requests = 0, epochs = 0, bytes = 0;
    bool  complete = false;
};

class UIPC_BACKEND_API Writer
{
  public:
    explicit Writer(const std::filesystem::path& fresh_output);
    ~Writer();
    void append(const Request& request);
    // Copies after producer completion; never calls the solver read-back hook.
    void    capture(const SimplexTrajectoryFilter::DetectInfo& info, Site site);
    void    close();
    Summary summary() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
class UIPC_BACKEND_API Reader
{
  public:
    explicit Reader(const std::filesystem::path& input);
    ~Reader();
    bool next(Request& request);
    // complete becomes true only after all records and footer counts agree.
    Summary summary() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
}  // namespace uipc::backend::cuda::simplex_query_trace
