#pragma once
#include <nonlinear_system/mas_pncg_ccd_math.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/codim_thickness.h>
#include <utils/distance.h>

namespace uipc::backend::cuda::pncg
{
inline UIPC_DEVICE void ccd_atomic_min(Float* address, Float value)
{
    auto* bits = reinterpret_cast<unsigned long long*>(address);
    auto  old  = atomicCAS(bits, 0ULL, 0ULL);
    while(value < __longlong_as_double(static_cast<long long>(old)))
    {
        auto assumed = old;
        old          = atomicCAS(bits,
                        assumed,
                        static_cast<unsigned long long>(__double_as_longlong(value)));
        if(old == assumed)
            break;
    }
}

inline UIPC_DEVICE IndexT ccd_domain(IndexT local_vertex,
                                     MASPreconditionerEngine::FrozenLevel0View base)
{
    if(base.real_to_part.size() && base.real_to_part(local_vertex) >= 0)
        return base.real_to_part(local_vertex) / MASPreconditionerEngine::BANKSIZE;
    // Every unpartitioned vertex gets its own domain, including all-diagonal FEM.
    return base.domain_count + local_vertex;
}

inline UIPC_DEVICE void ccd_limit_vertex(IndexT global_vertex,
                                         IndexT offset,
                                         MASPreconditionerEngine::FrozenLevel0View base,
                                         cuda_tool::CBufferView<IndexT> fixed,
                                         cuda_tool::BufferView<Float> fractions,
                                         Float                        fraction)
{
    const auto local = global_vertex - offset;
    if(local >= 0 && local < base.real_vertex_count && !fixed(local))
        ccd_atomic_min(&fractions(ccd_domain(local, base)), fraction);
}

// Counts: native pair fallbacks, locally clamped free vertices, invalid bounds.
template <bool PointTriangle>
static __global__ void local_ccd_pairs_kernel(cuda_tool::CBufferView<Vector2i> pairs,
                                              cuda_tool::CBufferView<IndexT> vertices,
                                              cuda_tool::CBufferView<Vector2i> edges,
                                              cuda_tool::CBufferView<Vector3i> triangles,
                                              cuda_tool::CBufferView<Vector3> positions,
                                              cuda_tool::CBufferView<Vector3> displacements,
                                              cuda_tool::CBufferView<Float> thicknesses,
                                              MASPreconditionerEngine::FrozenLevel0View base,
                                              cuda_tool::CBufferView<IndexT> fixed,
                                              cuda_tool::BufferView<Float> fractions,
                                              cuda_tool::BufferView<IndexT> counts,
                                              IndexT offset,
                                              Float  eta)
{
    const auto i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= pairs.size())
        return;
    auto     pair = pairs(i);
    Vector4i ids;
    if constexpr(PointTriangle)
    {
        auto triangle = triangles(pair(1));
        ids = Vector4i{vertices(pair(0)), triangle(0), triangle(1), triangle(2)};
    }
    else
    {
        auto a = edges(pair(0));
        auto b = edges(pair(1));
        ids    = Vector4i{a(0), a(1), b(0), b(1)};
    }
    Vector3 x[4], p[4];
    for(int j = 0; j < 4; ++j)
    {
        x[j] = positions(ids(j));
        p[j] = displacements(ids(j));
    }
    Float   thickness;
    CCDStep result;
    if constexpr(PointTriangle)
    {
        thickness = PT_thickness(thicknesses(ids(0)),
                                 thicknesses(ids(1)),
                                 thicknesses(ids(2)),
                                 thicknesses(ids(3)));
        result    = point_triangle_safe_step(x, p, thickness, eta);
    }
    else
    {
        thickness = EE_thickness(thicknesses(ids(0)),
                                 thicknesses(ids(1)),
                                 thicknesses(ids(2)),
                                 thicknesses(ids(3)));
        result    = edge_edge_safe_step(x, p, thickness, eta);
    }
    Float fraction = result.fraction;
    if(result.needs_fallback && isfinite(fraction) && fraction > 0)
    {
        atomicAdd(&counts(0), 1);
        // Coplanarity alone does not certify coplanar motion or finite thickness.
        Float toi = 1;
        bool  hit;
        if constexpr(PointTriangle)
            hit = distance::point_triangle_ccd(
                x[0], x[1], x[2], x[3], p[0], p[1], p[2], p[3], eta, thickness, 1000, toi);
        else
            hit = distance::edge_edge_ccd(
                x[0], x[1], x[2], x[3], p[0], p[1], p[2], p[3], eta, thickness, 1000, toi);
        fraction = hit ? toi : 1;
    }
    if(!isfinite(fraction) || fraction < 0 || fraction > 1)
    {
        atomicAdd(&counts(2), 1);
        fraction = 0;
    }
    for(int j = 0; j < 4; ++j)
        ccd_limit_vertex(ids(j), offset, base, fixed, fractions, fraction);
}

static __global__ void local_ccd_planes_kernel(
    cuda_tool::CBufferView<IndexT>            vertices,
    cuda_tool::CBufferView<Vector3>           positions,
    cuda_tool::CBufferView<Vector3>           displacements,
    cuda_tool::CBufferView<Float>             thicknesses,
    cuda_tool::CBufferView<IndexT>            contact_ids,
    cuda_tool::CBufferView<IndexT>            subscene_ids,
    cuda_tool::CDense2D<IndexT>               contact_mask,
    cuda_tool::CDense2D<IndexT>               subscene_mask,
    cuda_tool::CBufferView<Vector3>           plane_positions,
    cuda_tool::CBufferView<Vector3>           plane_normals,
    MASPreconditionerEngine::FrozenLevel0View base,
    cuda_tool::CBufferView<IndexT>            fixed,
    cuda_tool::BufferView<Float>              fractions,
    cuda_tool::BufferView<IndexT>             counts,
    IndexT                                    offset,
    IndexT                                    plane_offset)
{
    const auto i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= vertices.size())
        return;
    const auto v        = vertices(i);
    Float      fraction = 1;
    for(int j = 0; j < plane_positions.size(); ++j)
    {
        const auto plane = plane_offset + j;
        if(!contact_mask(contact_ids(v), contact_ids(plane))
           || !subscene_mask(subscene_ids(v), subscene_ids(plane)))
            continue;
        const Vector3 n       = plane_normals(j);
        const Float   closing = -n.dot(displacements(v));
        if(closing <= 0)
            continue;
        const Float gap = n.dot(positions(v) - plane_positions(j)) - thicknesses(v);
        // Match the native plane CCD's 10% separation margin.
        const Float bound = 0.9 * gap / closing;
        if(!isfinite(bound) || bound <= 0)
        {
            atomicAdd(&counts(2), 1);
            fraction = 0;
            break;
        }
        fraction = fmin(fraction, bound);
    }
    ccd_limit_vertex(v, offset, base, fixed, fractions, fraction);
}

static __global__ void local_ccd_scale_kernel(cuda_tool::CDenseVectorView<Float> raw,
                                              cuda_tool::DenseVectorView<Float> trial,
                                              MASPreconditionerEngine::FrozenLevel0View base,
                                              cuda_tool::CBufferView<IndexT> fixed,
                                              cuda_tool::CBufferView<Float> fractions,
                                              cuda_tool::BufferView<Float> vertex_fractions,
                                              cuda_tool::BufferView<IndexT> counts)
{
    const auto i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= base.real_vertex_count)
        return;
    const Float alpha   = fixed(i) ? 1 : fractions(ccd_domain(i, base));
    vertex_fractions(i) = alpha;
    if(alpha < 1)
        atomicAdd(&counts(1), 1);
    for(int j = 0; j < 3; ++j)
        trial(3 * i + j) = alpha * raw(3 * i + j);
}
}  // namespace uipc::backend::cuda::pncg
