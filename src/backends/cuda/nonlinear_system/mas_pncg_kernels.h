#pragma once
#include <contact_system/ipc_normal_surrogate.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <cuda_tool/cub.h>

namespace uipc::backend::cuda::pncg
{
using cuda_tool::BufferView;
using cuda_tool::CBufferView;
using cuda_tool::CDenseVectorView;
using cuda_tool::DenseVectorView;
using cuda_tool::VarView;
using Frozen = MASPreconditionerEngine::FrozenLevel0View;

__device__ inline bool same_contact(const IPCNormalSurrogate& a, const IPCNormalSurrogate& b)
{
    if(a.kind != b.kind || a.plane != b.plane || a.count != b.count)
        return false;
    for(int i = 0; i < 4; ++i)
        if(a.vertices[i] != b.vertices[i] || a.feature[i] != b.feature[i])
            return false;
    return true;
}

static __global__ void contact_hash_kernel(CBufferView<IPCNormalSurrogate> contacts,
                                           BufferView<unsigned long long> hashes,
                                           BufferView<IndexT> order)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= contacts.size())
        return;
    auto&              c = contacts(i);
    unsigned long long h = 14695981039346656037ull;
    h = (h ^ static_cast<unsigned int>(c.kind)) * 1099511628211ull;
    h = (h ^ static_cast<unsigned int>(c.plane)) * 1099511628211ull;
    for(int j = 0; j < 4; ++j)
    {
        h = (h ^ static_cast<unsigned int>(c.vertices[j])) * 1099511628211ull;
        h = (h ^ static_cast<unsigned int>(c.feature[j])) * 1099511628211ull;
    }
    hashes(i) = h;
    order(i)  = i;
}

__device__ inline int lower_bound(CBufferView<unsigned long long> keys, unsigned long long value)
{
    int a = 0, b = keys.size();
    while(a < b)
    {
        int m = a + (b - a) / 2;
        if(keys(m) < value)
            a = m + 1;
        else
            b = m;
    }
    return a;
}

static __global__ void validate_contacts_kernel(CBufferView<IPCNormalSurrogate> contacts,
                                                VarView<IndexT> error)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= contacts.size())
        return;
    auto& c = contacts(i);
    if(c.valid == 2)
        return;
    bool valid = c.valid == 1 && c.count >= 1 && c.count <= 4
                 && isfinite(c.stiffness) && c.stiffness >= 0;
    for(int j = 0; j < c.count && j < 4; ++j)
        valid &= c.vertices[j] >= 0 && c.q[j].allFinite();
    if(!valid)
        atomicMax(error.data(), 3);
}

static __global__ void contact_updates_kernel(CBufferView<IPCNormalSurrogate> current,
                                              CBufferView<unsigned long long> keys,
                                              CBufferView<IndexT> order,
                                              CBufferView<IPCNormalSurrogate> baseline,
                                              CBufferView<unsigned long long> base_keys,
                                              CBufferView<IndexT> base_order,
                                              CBufferView<IndexT> fixed,
                                              int                 vertex_offset,
                                              Float               cosine,
                                              BufferView<IPCNormalSurrogate> updates,
                                              VarView<IndexT> error,
                                              VarView<IndexT> column_count)
{
    int sorted = blockIdx.x * blockDim.x + threadIdx.x;
    if(sorted >= current.size())
        return;
    int  index = order(sorted);
    auto c     = current(index);
    auto out   = c;
    out.valid  = 0;
    if(c.valid != 1)
    {
        atomicMax(error.data(), c.valid == 2 ? 2 : 3);
        updates(index) = out;
        return;
    }
    int occurrence = 0;
    int start      = lower_bound(keys, keys(sorted));
    for(int j = start; j < sorted; ++j)
        if(same_contact(c, current(order(j))))
            ++occurrence;
    int found = -1;
    start     = lower_bound(base_keys, keys(sorted));
    for(int j = start; j < base_keys.size() && base_keys(j) == keys(sorted); ++j)
        if(same_contact(c, baseline(base_order(j))) && occurrence-- == 0)
        {
            found = base_order(j);
            break;
        }
    Float increment = c.stiffness;
    if(found >= 0)
    {
        auto& old  = baseline(found);
        Float norm = 0, old_norm = 0;
        int   normal = 0, old_normal = 0;
        for(int j = 0; j < c.count; ++j)
        {
            norm += c.q[j].squaredNorm();
            old_norm += old.q[j].squaredNorm();
            if(c.q[j].squaredNorm() > c.q[normal].squaredNorm())
                normal = j;
            if(old.q[j].squaredNorm() > old.q[old_normal].squaredNorm())
                old_normal = j;
        }
        bool aligned = old.valid == 1 && norm > 0 && old_norm > 0
                       && fabs(c.q[normal].dot(old.q[old_normal]))
                              >= cosine * c.q[normal].norm() * old.q[old_normal].norm()
                       && fabs(sqrt(norm / old_norm) - 1) <= 1e-3;
        if(aligned)
            for(int j = 0; j < c.count; ++j)
                aligned &= fabs(sqrt(c.q[j].squaredNorm() / norm)
                                - sqrt(old.q[j].squaredNorm() / old_norm))
                           <= 1e-3;
        if(aligned)
            increment = fmax(Float{0}, c.stiffness - old.stiffness);
    }
    Float scale     = sqrt(increment);
    Float magnitude = 0;
    for(int j = 0; j < c.count; ++j)
    {
        int v = c.vertices[j] - vertex_offset;
        if(v < 0 || v >= fixed.size())
        {
            atomicMax(error.data(), 3);
            out.q[j].setZero();
        }
        else
            out.q[j] = fixed(v) ? Vector3::Zero() : (scale * c.q[j]).eval();
        magnitude += out.q[j].squaredNorm();
    }
    out.valid = magnitude > 0 ? 1 : 0;
    if(out.valid)
        atomicAdd(column_count.data(), 1);
    updates(index) = out;
}

static __global__ void domain_keys_kernel(CBufferView<IPCNormalSurrogate> updates,
                                          CBufferView<int> real_to_part,
                                          int              vertex_offset,
                                          BufferView<unsigned long long> keys)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= updates.size())
        return;
    auto& c           = updates(i);
    int   previous[4] = {-1, -1, -1, -1};
    for(int j = 0; j < 4; ++j)
    {
        int domain = -1;
        if(c.valid == 1 && j < c.count)
        {
            int v = c.vertices[j] - vertex_offset;
            if(v >= 0 && v < real_to_part.size() && real_to_part(v) >= 0)
                domain = real_to_part(v) / 16;
        }
        bool unique = domain >= 0;
        for(int k = 0; k < j; ++k)
            unique &= previous[k] != domain;
        previous[j] = domain;
        keys(4 * i + j) = unique ? (static_cast<unsigned long long>(domain) << 32)
                                       | static_cast<unsigned int>(i) :
                                   ~0ull;
    }
}

static __global__ void select_domains_kernel(CBufferView<unsigned long long> keys,
                                             CBufferView<IPCNormalSurrogate> updates,
                                             CBufferView<int>   real_to_part,
                                             int                vertex_offset,
                                             int                domains,
                                             int                rank,
                                             BufferView<IndexT> selected,
                                             VarView<IndexT>    truncated)
{
    int d = blockIdx.x * blockDim.x + threadIdx.x;
    if(d >= domains)
        return;
    unsigned long long first = static_cast<unsigned long long>(d) << 32;
    int                begin = lower_bound(keys, first);
    int                end   = lower_bound(keys, first + (1ull << 32));
    for(int r = 0; r < rank; ++r)
    {
        Float best    = -1;
        int   best_id = -1;
        for(int j = begin; j < end; ++j)
        {
            int  id   = static_cast<unsigned int>(keys(j));
            bool used = false;
            for(int k = 0; k < r; ++k)
                used |= selected(d * rank + k) == id;
            if(used)
                continue;
            Float score = 0;
            auto& c     = updates(id);
            for(int k = 0; k < c.count; ++k)
            {
                int v = c.vertices[k] - vertex_offset;
                if(v >= 0 && v < real_to_part.size() && real_to_part(v) >= 0
                   && real_to_part(v) / 16 == d)
                    score += c.q[k].squaredNorm();
            }
            if(score > best)
            {
                best    = score;
                best_id = id;
            }
        }
        selected(d * rank + r) = best_id;
    }
    if(end - begin > rank)
        atomicAdd(truncated.data(), end - begin - rank);
}

__device__ inline Float packed_entry(const MASPreconditionerEngine::ClusterMatrixSym& matrix,
                                     int i,
                                     int j)
{
    int r = i / 3, c = j / 3;
    if(r <= c)
        return matrix.M[r * 16 - r * (r - 1) / 2 + c - r](i % 3, j % 3);
    return matrix.M[c * 16 - c * (c - 1) / 2 + r - c](j % 3, i % 3);
}

__device__ inline bool cholesky_solve(Float* matrix, Float* rhs, int n)
{
    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j <= i; ++j)
        {
            Float value = matrix[i * n + j];
            for(int k = 0; k < j; ++k)
                value -= matrix[i * n + k] * matrix[j * n + k];
            if(i == j)
            {
                if(!isfinite(value) || value <= 0)
                    return false;
                matrix[i * n + j] = sqrt(value);
            }
            else
                matrix[i * n + j] = value / matrix[j * n + j];
        }
    }
    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j < i; ++j)
            rhs[i] -= matrix[i * n + j] * rhs[j];
        rhs[i] /= matrix[i * n + i];
    }
    for(int i = n - 1; i >= 0; --i)
    {
        for(int j = i + 1; j < n; ++j)
            rhs[i] -= matrix[j * n + i] * rhs[j];
        rhs[i] /= matrix[i * n + i];
        if(!isfinite(rhs[i]))
            return false;
    }
    return true;
}

static __global__ void local_solve_kernel(Frozen base,
                                          CBufferView<IPCNormalSurrogate> updates,
                                          CBufferView<IndexT>     selected,
                                          CDenseVectorView<Float> gradient,
                                          DenseVectorView<Float>  z,
                                          int                     vertex_offset,
                                          int                     rank,
                                          VarView<IndexT>         error,
                                          VarView<IndexT> direct_fallbacks)
{
    int d = blockIdx.x, row = threadIdx.x;
    if(d >= base.domain_count)
        return;
    __shared__ Float u[48 * 8], w[48 * 8], t[48], answer[48];
    __shared__ Float capacitance[64], lambda[8], direct[48 * 48];
    __shared__ int   failed;
    if(row == 0)
        failed = 0;
    if(row < 48)
    {
        t[row] = 0;
        for(int col = 0; col < 48; ++col)
        {
            int v = base.part_to_real(d * 16 + col / 3);
            if(v >= 0)
                t[row] += packed_entry(base.base_inverses(d), row, col)
                          * gradient(v * 3 + col % 3);
        }
        int v = base.part_to_real(d * 16 + row / 3);
        for(int k = 0; k < rank; ++k)
        {
            Float value = 0;
            int   id    = selected(d * rank + k);
            if(id >= 0 && v >= 0)
                for(int j = 0; j < updates(id).count; ++j)
                    if(updates(id).vertices[j] - vertex_offset == v)
                        value += updates(id).q[j][row % 3];
            u[row * rank + k] = value;
        }
    }
    __syncthreads();
    if(row < 48)
        for(int k = 0; k < rank; ++k)
        {
            Float value = 0;
            for(int col = 0; col < 48; ++col)
                value += packed_entry(base.base_inverses(d), row, col) * u[col * rank + k];
            w[row * rank + k] = value;
        }
    __syncthreads();
    if(row == 0)
    {
        for(int i = 0; i < rank; ++i)
        {
            lambda[i] = 0;
            for(int r = 0; r < 48; ++r)
                lambda[i] += u[r * rank + i] * t[r];
            for(int j = 0; j < rank; ++j)
            {
                Float value = i == j ? 1 : 0;
                for(int r = 0; r < 48; ++r)
                    value += u[r * rank + i] * w[r * rank + j];
                capacitance[i * rank + j] = value;
            }
        }
        failed = !cholesky_solve(capacitance, lambda, rank);
    }
    __syncthreads();
    if(row < 48)
    {
        Float value = t[row];
        for(int k = 0; k < rank; ++k)
            value -= w[row * rank + k] * lambda[k];
        answer[row] = value;
    }
    __syncthreads();
    if(row == 0)
    {
        Float residual2 = 0, gradient2 = 0;
        Float projections[8] = {};
        for(int k = 0; k < rank; ++k)
            for(int i = 0; i < 48; ++i)
                projections[k] += u[i * rank + k] * answer[i];
        for(int i = 0; i < 48; ++i)
        {
            int   v   = base.part_to_real(d * 16 + i / 3);
            Float rhs = v >= 0 ? gradient(3 * v + i % 3) : 0;
            gradient2 += rhs * rhs;
            Float value = -rhs;
            for(int j = 0; j < 48; ++j)
                value += packed_entry(base.base_matrices(d), i, j) * answer[j];
            for(int k = 0; k < rank; ++k)
                value += u[i * rank + k] * projections[k];
            residual2 += value * value;
        }
        if(failed || !isfinite(residual2)
           || residual2 > 1e-14 * fmax(gradient2, Float{1e-60}))
        {
            for(int i = 0; i < 48; ++i)
            {
                int v     = base.part_to_real(d * 16 + i / 3);
                answer[i] = v >= 0 ? gradient(3 * v + i % 3) : 0;
                for(int j = 0; j < 48; ++j)
                {
                    Float a = packed_entry(base.base_matrices(d), i, j);
                    for(int k = 0; k < rank; ++k)
                        a += u[i * rank + k] * u[j * rank + k];
                    direct[i * 48 + j] = a;
                }
            }
            failed = !cholesky_solve(direct, answer, 48);
            atomicAdd(direct_fallbacks.data(), 1);
            for(int k = 0; k < rank; ++k)
            {
                projections[k] = 0;
                for(int i = 0; i < 48; ++i)
                    projections[k] += u[i * rank + k] * answer[i];
            }
            residual2 = 0;
            for(int i = 0; i < 48; ++i)
            {
                int   v     = base.part_to_real(d * 16 + i / 3);
                Float value = v >= 0 ? -gradient(3 * v + i % 3) : 0;
                for(int j = 0; j < 48; ++j)
                    value += packed_entry(base.base_matrices(d), i, j) * answer[j];
                for(int k = 0; k < rank; ++k)
                    value += u[i * rank + k] * projections[k];
                residual2 += value * value;
            }
            failed |= !isfinite(residual2)
                      || residual2 > 1e-14 * fmax(gradient2, Float{1e-60});
        }
        if(failed)
            atomicMax(error.data(), 1);
    }
    __syncthreads();
    if(row < 48)
    {
        int v = base.part_to_real(d * 16 + row / 3);
        if(v >= 0)
            z(3 * v + row % 3) += answer[row];
    }
}

static __global__ void mask_kernel(DenseVectorView<Float> vector, CBufferView<IndexT> fixed)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < vector.size() && fixed(i / 3))
        vector(i) = 0;
}

static __global__ void direction_kernel(CDenseVectorView<Float> z,
                                        CDenseVectorView<Float> history,
                                        DenseVectorView<Float>  direction,
                                        Float                   mu,
                                        Float                   nu)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < z.size())
        direction(i) = -mu * z(i) + nu * history(i);
}

static __global__ void scale_kernel(CDenseVectorView<Float> x, DenseVectorView<Float> y, Float scale)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < x.size())
        y(i) = scale * x(i);
}

static __global__ void residual_kernel(CDenseVectorView<Float> g,
                                       CBufferView<Float>      mass,
                                       CBufferView<IndexT>     fixed,
                                       CBufferView<Float>      length,
                                       BufferView<Float>       eta0,
                                       BufferView<Float>       terms,
                                       BufferView<Float>       ratios,
                                       BufferView<Float>       corrections,
                                       Float                   atol,
                                       Float                   rtol,
                                       bool                    initialize)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= mass.size())
        return;
    Float norm       = fixed(i) ? 0 : g.segment<3>(3 * i).as_eigen().norm();
    Float correction = fixed(i) ? 0 : norm / mass(i);
    Float eta        = correction / length(i);
    if(initialize)
        eta0(i) = eta;
    terms(i)       = fixed(i) ? 0 : norm * norm / mass(i);
    ratios(i)      = eta / fmax(atol, rtol * eta0(i));
    corrections(i) = correction;
}

static __global__ void projection_kernel(CBufferView<IPCNormalSurrogate> updates,
                                         CDenseVectorView<Float> z,
                                         CDenseVectorView<Float> p,
                                         int                     vertex_offset,
                                         BufferView<Vector3>     projections)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= updates.size())
        return;
    Float uz = 0, up = 0;
    auto& c = updates(i);
    if(c.valid == 1)
        for(int j = 0; j < c.count; ++j)
        {
            int v = c.vertices[j] - vertex_offset;
            uz += c.q[j].dot(z.segment<3>(3 * v).as_eigen());
            up += c.q[j].dot(p.segment<3>(3 * v).as_eigen());
        }
    projections(i) = Vector3{uz * uz, uz * up, up * up};
}

struct ProjectionSum
{
    __host__ __device__ Vector3 operator()(const Vector3& a, const Vector3& b) const
    {
        return (a + b).eval();
    }
};
}  // namespace uipc::backend::cuda::pncg
