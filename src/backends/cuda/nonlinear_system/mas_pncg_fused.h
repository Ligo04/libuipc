#pragma once
#include <nonlinear_system/mas_pncg_kernels.h>
#include <nonlinear_system/mas_pncg_math.h>
#include <cuda_tool/graph.h>
#include <cub/block/block_reduce.cuh>
#include <array>
#include <algorithm>

namespace uipc::backend::cuda::pncg
{
struct ResidualSums
{
    Float  sum = 0, ratio = 0, correction = 0;
    IndexT invalid = 0;
};

struct DirectionSums
{
    Float  gz = 0, old_gz = 0, a = 0, b = 0, c = 0, gp = 0;
    IndexT invalid = 0, columns = 0;
    Float  z_squared_norm = 0;
};

struct DirectionResult
{
    Float  mu = 0, nu = 0, slope = 0;
    Float  a = 0, b = 0, c = 0, gz = 0, gp = 0;
    IndexT valid = 0, used_history = 0, update_columns = 0;
    Float  preconditioned_norm = 0, restart_ratio = 0;
    IndexT restart_next = 0;
};

namespace fused
{
    constexpr int block_size = 256;

    struct ResidualOp
    {
        __device__ ResidualSums operator()(const ResidualSums& a, const ResidualSums& b) const
        {
            return {a.sum + b.sum,
                    fmax(a.ratio, b.ratio),
                    fmax(a.correction, b.correction),
                    a.invalid | b.invalid};
        }
    };

    struct DirectionOp
    {
        __device__ DirectionSums operator()(const DirectionSums& a, const DirectionSums& b) const
        {
            return {a.gz + b.gz,
                    a.old_gz + b.old_gz,
                    a.a + b.a,
                    a.b + b.b,
                    a.c + b.c,
                    a.gp + b.gp,
                    a.invalid | b.invalid,
                    a.columns + b.columns,
                    a.z_squared_norm + b.z_squared_norm};
        }
    };

    template <typename T, typename Op>
    __device__ void block_result(T value, BufferView<T> partials, Op op)
    {
        using Reduce = cub::BlockReduce<T, block_size>;
        __shared__ typename Reduce::TempStorage storage;
        const T result = Reduce(storage).Reduce(value, op);
        if(threadIdx.x == 0)
            partials(blockIdx.x) = result;
    }

    template <typename T, typename Op>
    static __global__ void finish(CBufferView<T> partials, VarView<T> result, Op op)
    {
        T value{};
        for(int i = threadIdx.x; i < partials.size(); i += blockDim.x)
            value = op(value, partials(i));
        using Reduce = cub::BlockReduce<T, block_size>;
        __shared__ typename Reduce::TempStorage storage;
        const T sum = Reduce(storage).Reduce(value, op);
        if(threadIdx.x == 0)
            *result = sum;
    }

    static __global__ void residual(CDenseVectorView<Float>  g,
                                    CBufferView<Float>       mass,
                                    CBufferView<IndexT>      fixed,
                                    CBufferView<Float>       length,
                                    BufferView<Float>        eta0,
                                    Float                    atol,
                                    Float                    rtol,
                                    bool                     initialize,
                                    BufferView<ResidualSums> partials)
    {
        const int    i = blockIdx.x * blockDim.x + threadIdx.x;
        ResidualSums value{};
        if(i < mass.size())
        {
            const Float norm = fixed(i) ? 0 : g.segment<3>(3 * i).as_eigen().norm();
            value.correction = fixed(i) ? 0 : norm / mass(i);
            const Float eta  = value.correction / length(i);
            if(initialize)
                eta0(i) = eta;
            value.sum     = fixed(i) ? 0 : norm * norm / mass(i);
            value.ratio   = eta / fmax(atol, rtol * eta0(i));
            value.invalid = !isfinite(value.sum) || !isfinite(value.ratio)
                            || !isfinite(value.correction);
        }
        block_result(value, partials, ResidualOp{});
    }

    static __global__ void direction_sums(CDenseVectorView<Float>   g,
                                          CDenseVectorView<Float>   z,
                                          CDenseVectorView<Float>   old_z,
                                          CDenseVectorView<Float>   p,
                                          CDenseVectorView<Float>   hz,
                                          CDenseVectorView<Float>   hp,
                                          const IPCNormalSurrogate* updates,
                                          cuda_tool::CVarView<IndexT> update_count,
                                          int  vertex_offset,
                                          bool history,
                                          bool defer_restart,
                                          BufferView<DirectionSums> partials)
    {
        const int     i = blockIdx.x * blockDim.x + threadIdx.x;
        DirectionSums value{};
        if(i < g.size())
        {
            value.gz = g(i) * z(i);
            value.a  = z(i) * hz(i);
            if(history || defer_restart)
                value.old_gz = g(i) * old_z(i);
            if(defer_restart)
                value.z_squared_norm = z(i) * z(i);
            if(history)
            {
                value.b  = z(i) * hp(i);
                value.c  = p(i) * hp(i);
                value.gp = g(i) * p(i);
            }
        }
        if(i < *update_count)
        {
            const auto& c = updates[i];
            value.columns = c.valid == 1;
            Float uz = 0, up = 0;
            if(c.valid == 1)
                for(int j = 0; j < c.count; ++j)
                {
                    const int v = c.vertices[j] - vertex_offset;
                    uz += c.q[j].dot(z.segment<3>(3 * v).as_eigen());
                    if(history)
                        up += c.q[j].dot(p.segment<3>(3 * v).as_eigen());
                }
            value.a += uz * uz;
            value.b += uz * up;
            value.c += up * up;
        }
        // Nonfinite history curvature may still use the original 1D direct
        // fallback. Only the quantities needed by that fallback veto it.
        value.invalid =
            !isfinite(value.gz) || !isfinite(value.old_gz) || !isfinite(value.a);
        block_result(value, partials, DirectionOp{});
    }

    static __global__ void solve_subspace(cuda_tool::CVarView<DirectionSums> sums,
                                          bool  history,
                                          Float threshold,
                                          bool  defer_restart,
                                          VarView<DirectionResult> result)
    {
        DirectionResult out{};
        const auto      s       = *sums;
        out.a                   = s.a;
        out.b                   = s.b;
        out.c                   = s.c;
        out.gz                  = s.gz;
        out.gp                  = s.gp;
        out.update_columns      = s.columns;
        out.preconditioned_norm = sqrt(s.z_squared_norm);
        if(!s.invalid && isfinite(s.gz) && s.gz > 0)
        {
            out.restart_ratio = fabs(s.old_gz) / s.gz;
            out.restart_next  = out.restart_ratio > threshold;
        }
        if(!s.invalid && isfinite(s.gz) && s.gz > 0
           && (defer_restart || !history
               || (isfinite(s.old_gz) && fabs(s.old_gz) / s.gz <= threshold)))
        {
            // The same direct 2x2 solve as the host reference; no inner PCG.
            const auto solution = solve_pncg_subspace(s.a, s.b, s.c, s.gz, s.gp, history);
            out.mu           = solution.mu;
            out.nu           = solution.nu;
            out.valid        = solution.valid;
            out.used_history = solution.used_history;
        }
        *result = out;
    }

    static __global__ void direction_and_slope(CDenseVectorView<Float> g,
                                               CDenseVectorView<Float> z,
                                               CDenseVectorView<Float> p,
                                               cuda_tool::CVarView<DirectionResult> parameters,
                                               DenseVectorView<Float> direction,
                                               BufferView<Float>      partials)
    {
        const int  i     = blockIdx.x * blockDim.x + threadIdx.x;
        Float      value = 0;
        const auto s     = *parameters;
        if(i < direction.size())
        {
            const Float d = s.valid ? -s.mu * z(i) + s.nu * p(i) : 0;
            direction(i)  = d;
            value         = g(i) * d;
        }
        using Reduce = cub::BlockReduce<Float, block_size>;
        __shared__ typename Reduce::TempStorage storage;
        const Float sum = Reduce(storage).Sum(value);
        if(threadIdx.x == 0)
            partials(blockIdx.x) = sum;
    }

    static __global__ void finish_slope(CBufferView<Float>       partials,
                                        VarView<DirectionResult> result)
    {
        Float value = 0;
        for(int i = threadIdx.x; i < partials.size(); i += blockDim.x)
            value += partials(i);
        using Reduce = cub::BlockReduce<Float, block_size>;
        __shared__ typename Reduce::TempStorage storage;
        const Float sum = Reduce(storage).Sum(value);
        if(threadIdx.x == 0)
        {
            result->slope = sum;
            result->valid = result->valid && isfinite(sum) && sum < 0;
        }
    }

    static __global__ void accept(CDenseVectorView<Float> direction,
                                  CDenseVectorView<Float> z,
                                  DenseVectorView<Float>  p,
                                  DenseVectorView<Float>  old_z,
                                  Float                   fraction)
    {
        const int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i < p.size())
        {
            p(i)     = fraction * direction(i);
            old_z(i) = z(i);
        }
    }

    struct GraphKey
    {
        std::array<const void*, 13> pointers{};
        std::array<SizeT, 4>        sizes{};
        std::array<Float, 2>        scalars{};
        bool                        operator==(const GraphKey&) const = default;
    };

    struct CachedGraph
    {
        cuda_tool::GraphCapture graph;
        GraphKey                key;
        SizeT                   captures = 0, replays = 0, fallbacks = 0;

        template <typename Body>
        void run(bool enabled, const GraphKey& current, Body&& body)
        {
            if(enabled && !graph.disabled())
            {
                if(!graph.ready() || !(key == current))
                {
                    key               = current;
                    const auto result = graph.capture(body);
                    if(result == cuda_tool::GraphCapture::Result::Ok)
                        ++captures;
                    else
                        ++fallbacks;
                }
                if(graph.ready())
                {
                    // Replay on the producer stream, so SpMV/gradient work is
                    // ordered before consumption without an extra host fence.
                    CUDA_TOOL_CHECK(graph.launch());
                    ++replays;
                    return;
                }
            }
            body(nullptr);
            CUDA_TOOL_CHECK(cudaGetLastError());
        }
    };
}  // namespace fused

// Own scratch outside captured regions. No CUB allocation or host transfers
// are captured; each operation returns one consolidated readback.
class FusedOperations
{
  public:
    ResidualSums residual(CDenseVectorView<Float> g,
                          CBufferView<Float>      mass,
                          CBufferView<IndexT>     fixed,
                          CBufferView<Float>      length,
                          BufferView<Float>       eta0,
                          Float                   atol,
                          Float                   rtol,
                          bool                    initialize,
                          bool                    graph)
    {
        const int blocks = block_count(mass.size());
        m_residual_partials.resize_discard(blocks);
        fused::GraphKey key{
            {g.data(),
             mass.data(),
             fixed.data(),
             length.data(),
             eta0.data(),
             m_residual_partials.data(),
             m_residual.data()},
            {static_cast<SizeT>(g.size()), mass.size(), static_cast<SizeT>(blocks)},
            {atol, rtol}};
        m_residual_graph[initialize].run(
            graph,
            key,
            [&](cudaStream_t stream)
            {
                if(blocks)
                    fused::residual<<<blocks, fused::block_size, 0, stream>>>(
                        g,
                        mass,
                        fixed,
                        length,
                        eta0,
                        atol,
                        rtol,
                        initialize,
                        m_residual_partials.view());
                fused::finish<<<1, fused::block_size, 0, stream>>>(
                    m_residual_partials.cview(), m_residual.view(), fused::ResidualOp{});
            });
        return m_residual;
    }

    DirectionResult direction(CDenseVectorView<Float>         g,
                              CDenseVectorView<Float>         z,
                              CDenseVectorView<Float>         old_z,
                              CDenseVectorView<Float>         p,
                              CDenseVectorView<Float>         hz,
                              CDenseVectorView<Float>         hp,
                              CBufferView<IPCNormalSurrogate> updates,
                              int                             vertex_offset,
                              bool                            history,
                              Float                           threshold,
                              DenseVectorView<Float>          direction,
                              bool                            graph,
                              bool defer_restart = false)
    {
        const int blocks = block_count(std::max<SizeT>(g.size(), updates.size()));
        const int slope_blocks = block_count(g.size());
        m_dot_partials.resize_discard(blocks);
        m_slope_partials.resize_discard(slope_blocks);
        // Counts change as contacts activate. Reuse the graph when its
        // allocation and launch shape still cover the active contacts.
        const IndexT update_count = updates.size();
        m_active_updates.copy_from(update_count);
        fused::GraphKey key{{g.data(),
                             z.data(),
                             old_z.data(),
                             p.data(),
                             hz.data(),
                             hp.data(),
                             updates.data(),
                             direction.data(),
                             m_dot_partials.data(),
                             m_slope_partials.data(),
                             m_sums.data(),
                             m_direction.data(),
                             m_active_updates.data()},
                            {static_cast<SizeT>(g.size()),
                             static_cast<SizeT>(blocks),
                             static_cast<SizeT>(vertex_offset)},
                            {threshold, Float(defer_restart)}};
        m_direction_graph[history].run(
            graph,
            key,
            [&](cudaStream_t stream)
            {
                if(blocks)
                    fused::direction_sums<<<blocks, fused::block_size, 0, stream>>>(
                        g,
                        z,
                        old_z,
                        p,
                        hz,
                        hp,
                        updates.data(),
                        m_active_updates.cview(),
                        vertex_offset,
                        history,
                        defer_restart,
                        m_dot_partials.view());
                fused::finish<<<1, fused::block_size, 0, stream>>>(
                    m_dot_partials.cview(), m_sums.view(), fused::DirectionOp{});
                fused::solve_subspace<<<1, 1, 0, stream>>>(m_sums.cview(),
                                                           history,
                                                           threshold,
                                                           defer_restart,
                                                           m_direction.view());
                if(slope_blocks)
                    fused::direction_and_slope<<<slope_blocks, fused::block_size, 0, stream>>>(
                        g,
                        z,
                        p,
                        m_direction.cview(),
                        direction,
                        m_slope_partials.view());
                fused::finish_slope<<<1, fused::block_size, 0, stream>>>(
                    m_slope_partials.cview(), m_direction.view());
            });
        return m_direction;
    }

    void accept(CDenseVectorView<Float> direction,
                CDenseVectorView<Float> z,
                DenseVectorView<Float>  p,
                DenseVectorView<Float>  old_z,
                Float                   fraction)
    {
        const int blocks = block_count(p.size());
        if(blocks)
            fused::accept<<<blocks, fused::block_size>>>(direction, z, p, old_z, fraction);
        CUDA_TOOL_CHECK(cudaGetLastError());
    }

    SizeT graph_captures() const
    {
        return graph_count(&fused::CachedGraph::captures);
    }
    SizeT graph_replays() const
    {
        return graph_count(&fused::CachedGraph::replays);
    }
    SizeT graph_fallbacks() const
    {
        return graph_count(&fused::CachedGraph::fallbacks);
    }

  private:
    static int block_count(SizeT n)
    {
        return (n + fused::block_size - 1) / fused::block_size;
    }
    SizeT graph_count(SizeT fused::CachedGraph::*field) const
    {
        SizeT count = 0;
        for(const auto& graph : m_residual_graph)
            count += graph.*field;
        for(const auto& graph : m_direction_graph)
            count += graph.*field;
        return count;
    }
    cuda_tool::DeviceBuffer<ResidualSums>  m_residual_partials;
    cuda_tool::DeviceVar<ResidualSums>     m_residual;
    cuda_tool::DeviceBuffer<DirectionSums> m_dot_partials;
    cuda_tool::DeviceVar<DirectionSums>    m_sums;
    cuda_tool::DeviceVar<IndexT>           m_active_updates;
    cuda_tool::DeviceBuffer<Float>         m_slope_partials;
    cuda_tool::DeviceVar<DirectionResult>  m_direction;
    // Separate first/history graphs avoid re-capture at normal restarts.
    std::array<fused::CachedGraph, 2> m_residual_graph, m_direction_graph;
};
}  // namespace uipc::backend::cuda::pncg
