#pragma once
#include <collision_detection/aabb.h>
#include <cmath>
#include <limits>

namespace uipc::backend::cuda
{
// A traversal thread prepares its segment once, including reciprocal directions.
class SegmentAABBQuery
{
  public:
    UIPC_GENERIC SegmentAABBQuery() = default;
    UIPC_GENERIC SegmentAABBQuery(const Vector3& from, const Vector3& to, Float radius)
        : m_from(from)
        , m_to(to)
        , m_radius(radius)
    {
        m_valid = std::isfinite(radius) && radius >= 0.0;
        for(int axis = 0; axis < 3; ++axis)
        {
            const Float direction = to[axis] - from[axis];
            m_inverse[axis]       = direction == 0.0 ? 0.0 : 1.0 / direction;
            m_valid &= std::isfinite(from[axis]) && std::isfinite(to[axis])
                       && std::isfinite(direction) && std::isfinite(m_inverse[axis]);
        }
    }

    // Closed segment against a box enlarged by an L-infinity radius. The
    // additional radius accounts for the caller's bound-construction error.
    UIPC_GENERIC bool intersects(const AABB& box, Float additional_radius = 0.0) const
    {
#ifdef __CUDA_ARCH__
        const Float radius = __dadd_ru(m_radius, additional_radius);
#else
        const Float radius = std::nextafter(m_radius + additional_radius,
                                            std::numeric_limits<Float>::infinity());
#endif
        if(!m_valid || !std::isfinite(radius) || additional_radius < 0.0)
            return true;
        Vector3 lower, upper;
        bool    from_inside = true, to_inside = true;
        for(int axis = 0; axis < 3; ++axis)
        {
            const float lo = box.min()[axis], hi = box.max()[axis];
            if(!std::isfinite(lo) || !std::isfinite(hi))
                return true;
            const Float outer_lo =
                std::nextafter(lo, -std::numeric_limits<float>::infinity());
            const Float outer_hi =
                std::nextafter(hi, std::numeric_limits<float>::infinity());
#ifdef __CUDA_ARCH__
            lower[axis] = __dsub_rd(outer_lo, radius);
            upper[axis] = __dadd_ru(outer_hi, radius);
#else
            lower[axis] = std::nextafter(outer_lo - radius,
                                         -std::numeric_limits<Float>::infinity());
            upper[axis] = std::nextafter(outer_hi + radius,
                                         std::numeric_limits<Float>::infinity());
#endif
            from_inside &= m_from[axis] >= lower[axis] && m_from[axis] <= upper[axis];
            to_inside &= m_to[axis] >= lower[axis] && m_to[axis] <= upper[axis];
        }
        if(from_inside || to_inside)
            return true;
        Float enter = 0.0, exit = 1.0;
        for(int axis = 0; axis < 3; ++axis)
        {
            if(m_inverse[axis] == 0.0)
            {
                if(m_from[axis] < lower[axis] || m_from[axis] > upper[axis])
                    return false;
                continue;
            }
            const Float t0   = (lower[axis] - m_from[axis]) * m_inverse[axis];
            const Float t1   = (upper[axis] - m_from[axis]) * m_inverse[axis];
            const Float near = t0 < t1 ? t0 : t1, far = t0 > t1 ? t0 : t1;
            // Cover subtraction, reciprocal and product rounding without
            // nextafter calls in the per-node parameter interval.
            const Float near_margin =
                8.0 * std::numeric_limits<Float>::epsilon() * (std::abs(near) + 1.0);
            const Float far_margin =
                8.0 * std::numeric_limits<Float>::epsilon() * (std::abs(far) + 1.0);
            enter = enter > near - near_margin ? enter : near - near_margin;
            exit  = exit < far + far_margin ? exit : far + far_margin;
            if(enter > exit)
                return false;
        }
        return true;
    }

  private:
    Vector3 m_from = Vector3::Zero(), m_to = Vector3::Zero(),
            m_inverse = Vector3::Zero();
    Float m_radius    = 0.0;
    bool  m_valid     = false;
};

inline UIPC_GENERIC bool segment_aabb_intersects(const Vector3& from,
                                                 const Vector3& to,
                                                 const AABB&    box,
                                                 Float          radius = 0.0)
{
    return SegmentAABBQuery(from, to, radius).intersects(box);
}
}  // namespace uipc::backend::cuda
