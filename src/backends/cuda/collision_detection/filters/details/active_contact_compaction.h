#pragma once
#include <cuda_tool/cub.h>
#include <uipc/common/type_define.h>
#include <array>
#include <cstdio>
#include <stdexcept>

namespace uipc::backend::cuda::details
{
// Bounds classification/selection storage independently of the broad-phase
// candidate count. This is an allocation batch size, not a contact threshold.
inline constexpr SizeT ActiveContactChunkSize = 262144;

class ActiveContactCompaction
{
  public:
    ActiveContactCompaction(cuda_tool::DeviceBuffer<Vector2i>& pps,
                            cuda_tool::DeviceBuffer<Vector3i>& pes,
                            cuda_tool::DeviceBuffer<Vector4i>& pts,
                            cuda_tool::DeviceBuffer<Vector4i>& ees,
                            cuda_tool::DeviceBuffer<IndexT>&   counts)
        : m_pps(pps)
        , m_pes(pes)
        , m_pts(pts)
        , m_ees(ees)
        , m_counts(counts)
    {
    }

    void reset()
    {
        m_pps.resize_discard(0);
        m_pes.resize_discard(0);
        m_pts.resize_discard(0);
        m_ees.resize_discard(0);
    }

    // DeviceSelect preserves input order, including duplicates. Append chunks
    // in source order to reproduce one stable selection over the entire input.
    void append(cuda_tool::CBufferView<Vector2i> pps,
                cuda_tool::CBufferView<Vector3i> pes = {},
                cuda_tool::CBufferView<Vector4i> pts = {},
                cuda_tool::CBufferView<Vector4i> ees = {})
    {
        const std::array<SizeT, 4> sizes{pps.size(), pes.size(), pts.size(), ees.size()};
        for(SizeT size : sizes)
            if(size > ActiveContactChunkSize)
                throw std::length_error("Active contact chunk exceeds its storage bound");
        if(pps.size() + pes.size() + pts.size() + ees.size() == 0)
            return;

        const std::array<SizeT, 4> written{
            m_pps.size(), m_pes.size(), m_pts.size(), m_ees.size()};
        m_counts.resize_discard(4);
        m_counts.fill(0);
        select(pps, m_pps, 0, "PPs");
        select(pes, m_pes, 1, "PEs");
        select(pts, m_pts, 2, "PTs");
        select(ees, m_ees, 3, "EEs");

        std::array<IndexT, 4> selected{};
        m_counts.copy_to(selected.data());
        for(SizeT i = 0; i < selected.size(); ++i)
            if(selected[i] < 0 || static_cast<SizeT>(selected[i]) > sizes[i])
                throw std::runtime_error("Invalid active contact selection count");

        m_pps.resize_preserve(written[0] + selected[0]);
        m_pes.resize_preserve(written[1] + selected[1]);
        m_pts.resize_preserve(written[2] + selected[2]);
        m_ees.resize_preserve(written[3] + selected[3]);
    }

    // NVCC's generated CUB launch stubs name the predicate type.
    struct IsActive
    {
        template <typename T>
        CUB_RUNTIME_FUNCTION bool operator()(const T& contact) const
        {
            return contact(0) != -1;
        }
    };

  private:
    template <typename T>
    void select(cuda_tool::CBufferView<T>   input,
                cuda_tool::DeviceBuffer<T>& output,
                SizeT                       slot,
                const char*                 name)
    {
        if(input.size() == 0)
            return;
        const SizeT written  = output.size();
        const SizeT required = written + input.size();
        try
        {
            output.resize_preserve(required);
        }
        catch(const std::exception& error)
        {
            size_t free = 0, total = 0;
            cudaMemGetInfo(&free, &total);
            std::fprintf(stderr,
                         "[PNCG-ALLOCATION-FAILURE] buffer=%s requested_elements=%zu element_bytes=%zu old_capacity=%zu free=%zu total=%zu error=%s\n",
                         name,
                         required,
                         sizeof(T),
                         output.capacity(),
                         free,
                         total,
                         error.what());
            std::fflush(stderr);
            throw;
        }
        cuda_tool::DeviceSelect().If(input.data(),
                                     output.data() + written,
                                     m_counts.data() + slot,
                                     static_cast<int>(input.size()),
                                     IsActive{});
    }

    cuda_tool::DeviceBuffer<Vector2i>& m_pps;
    cuda_tool::DeviceBuffer<Vector3i>& m_pes;
    cuda_tool::DeviceBuffer<Vector4i>& m_pts;
    cuda_tool::DeviceBuffer<Vector4i>& m_ees;
    cuda_tool::DeviceBuffer<IndexT>&   m_counts;
};
}  // namespace uipc::backend::cuda::details
