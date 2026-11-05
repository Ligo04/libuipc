#include <app/app.h>
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <cuda_tool/graph.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace test_single_ef_morton
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct = uipc::backend::cuda_tool;

struct AcceptTiedNode
{
    __device__ bool operator()(const InfoStacklessBVH::NodePredInfo&) const
    {
        return true;
    }
};

struct AcceptTiedLeaf
{
    __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo&) const
    {
        return true;
    }
};
}  // namespace test_single_ef_morton

TEST_CASE("single EF shared tree handles equal Morton codes and graph replay",
          "[cuda][single_ef_bvh][morton_ties]")
{
    using namespace test_single_ef_morton;
    for(int count : {1, 2, 31, 32, 33, 257})
    {
        CAPTURE(count);
        AABB box;
        box.extend(Eigen::Vector3f::Zero()).extend(Eigen::Vector3f::Ones());
        std::vector<AABB>        host_boxes(count, box);
        std::vector<IndexT>      tags(count, 0);
        ct::DeviceBuffer<AABB>   boxes(count);
        ct::DeviceBuffer<IndexT> ids(count);
        boxes.view().copy_from(host_boxes.data());
        ids.view().copy_from(tags.data());
        InfoStacklessBVH::Impl tree;
        tree.build(boxes, ids, ids);

        std::vector<InfoStacklessBVH::Node> nodes(tree.nodes.size());
        tree.nodes.view().copy_to(nodes.data());
        std::vector<std::pair<int, int>> pending{{0, 0}};
        int                              visited = 0, leaves = 0, max_depth = 0;
        while(!pending.empty())
        {
            const auto [id, depth] = pending.back();
            pending.pop_back();
            REQUIRE(id >= 0);
            REQUIRE(id < nodes.size());
            REQUIRE(++visited <= nodes.size());
            max_depth = std::max(max_depth, depth);
            if(nodes[id].lc < 0)
                ++leaves;
            else
            {
                const auto left = nodes[id].lc;
                pending.emplace_back(left, depth + 1);
                pending.emplace_back(nodes[left].escape, depth + 1);
            }
        }
        CHECK(leaves == count);
        int logarithmic_depth = 0;
        while((1 << logarithmic_depth) < count)
            ++logarithmic_depth;
        CHECK(max_depth <= logarithmic_depth);

        const int                  expected_count = count * (count - 1) / 2;
        ct::DeviceBuffer<Vector2i> pairs(std::max(1, expected_count));
        ct::DeviceVar<int>         live_count;
        ct::GraphCapture           graph;
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        REQUIRE(graph.capture(
                    [&](cudaStream_t stream)
                    {
                        CUDA_TOOL_CHECK(
                            cudaMemsetAsync(live_count.data(), 0, sizeof(int), stream));
                        tree.stacklessSelf(AcceptTiedNode{},
                                           AcceptTiedLeaf{},
                                           live_count.view(),
                                           pairs.view(),
                                           stream);
                    })
                == ct::GraphCapture::Result::Ok);
        for(int replay = 0; replay < 3; ++replay)
        {
            REQUIRE(graph.launch(nullptr) == cudaSuccess);
            REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
            REQUIRE(static_cast<int>(live_count) == expected_count);
            std::vector<Vector2i> output(expected_count);
            if(expected_count)
                pairs.view(0, expected_count).copy_to(output.data());
            std::vector<std::pair<int, int>> canonical;
            for(const auto& pair : output)
            {
                REQUIRE(pair[0] >= 0);
                REQUIRE(pair[0] < count);
                REQUIRE(pair[1] >= 0);
                REQUIRE(pair[1] < count);
                REQUIRE(pair[0] != pair[1]);
                canonical.emplace_back(std::min(pair[0], pair[1]),
                                       std::max(pair[0], pair[1]));
            }
            std::sort(canonical.begin(), canonical.end());
            CHECK(std::adjacent_find(canonical.begin(), canonical.end())
                  == canonical.end());
        }
    }
}

TEST_CASE("single EF swept endpoints use accepted-step rounding", "[cuda][single_ef_bvh][step_rounding]")
{
    using namespace test_single_ef_morton;
    // Rounded multiplication gives exactly 1, followed by -1 + 1 = 0.
    // An FMA instead retains the -2^-54 product residual.
    const Float               delta    = std::ldexp(Float{1}, -27);
    const Vector3             position = Vector3::Constant(-1);
    const Vector3             motion   = Vector3::Constant(1 - delta);
    const Float               d_hat = 0.01, thickness = 0;
    ct::DeviceBuffer<Vector3> positions(1), displacements(1);
    ct::DeviceBuffer<Float>   d_hats(1), thicknesses(1);
    ct::DeviceBuffer<float4>  starts(1), steps(1), ends(1);
    positions.view().copy_from(&position);
    displacements.view().copy_from(&motion);
    d_hats.view().copy_from(&d_hat);
    thicknesses.view().copy_from(&thickness);
    TrajectoryPrefix prefix;
    prefix.accepted_step_endpoints = true;
    prefix.vertices                = 1;
    prefix.positions               = positions.data();
    prefix.displacements           = displacements.data();
    prefix.d_hats                  = d_hats.data();
    prefix.thicknesses             = thicknesses.data();
    prefix.alpha                   = 1 + delta;
    prefix.f32_positions           = starts.data();
    prefix.f32_steps               = steps.data();
    prefix.f32_ends                = ends.data();
    launch_trajectory_prefix(prefix, nullptr);
    float4 actual;
    ends.copy_to(&actual);
    CHECK(actual.x == 0.0f);
    CHECK(actual.y == 0.0f);
    CHECK(actual.z == 0.0f);
}
