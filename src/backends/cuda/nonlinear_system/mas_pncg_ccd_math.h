#pragma once
#include <type_define.h>
#include <utils/distance/distance_flagged.h>
#include <cmath>
#include <limits>

namespace uipc::backend::cuda::pncg
{
struct CCDStep
{
    Float fraction       = 0;
    bool  needs_fallback = true;
};

struct CubicPolynomial
{
    Float d = 0, c = 0, b = 0, a = 0;
};

namespace cccd_detail
{
    inline UIPC_GENERIC Float value(const CubicPolynomial& f, Float t)
    {
        return std::fma(std::fma(std::fma(f.a, t, f.b), t, f.c), t, f.d);
    }

    // Treat roundoff-sized evaluations as an unsafe boundary, including a
    // tangency whose sign is the same on both sides of the root.
    inline UIPC_GENERIC int sign(const CubicPolynomial& f, Float t)
    {
        const Float magnitude =
            ((std::abs(f.a) * t + std::abs(f.b)) * t + std::abs(f.c)) * t
            + std::abs(f.d);
        const Float tolerance = 64 * std::numeric_limits<Float>::epsilon() * magnitude;
        const Float y = value(f, t);
        return y > tolerance ? 1 : (y < -tolerance ? -1 : 0);
    }

    inline UIPC_GENERIC Float safe_bisect(const CubicPolynomial& f, Float safe, Float unsafe, int safe_sign)
    {
        for(int i = 0; i < 16; ++i)
        {
            const Float middle = safe + (unsafe - safe) * 0.5;
            if(sign(f, middle) == safe_sign)
                safe = middle;
            else
                unsafe = middle;
        }
        return safe;
    }

    inline UIPC_GENERIC bool finite(const Vector3& v)
    {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    }
}  // namespace cccd_detail

// Supplement Algorithms 1/2, with explicit lower-degree and uncertain-root
// handling. The caller must independently certify [0, lower_bound] as safe;
// the polynomial only bounds subsequent coplanarity, not finite thickness.
// needs_fallback means no extension of that supplied bound is certified.
inline UIPC_GENERIC CCDStep cubic_safe_step(CubicPolynomial f, Float lower_bound)
{
    if(!std::isfinite(lower_bound) || lower_bound < 0 || lower_bound > 1
       || !std::isfinite(f.a) || !std::isfinite(f.b) || !std::isfinite(f.c)
       || !std::isfinite(f.d))
        return {};
    if(lower_bound == 1)
        return {1, false};
    const Float scale = std::fmax(std::fmax(std::abs(f.a), std::abs(f.b)),
                                  std::fmax(std::abs(f.c), std::abs(f.d)));
    if(scale == 0)
        return {lower_bound, true};
    f.a /= scale;
    f.b /= scale;
    f.c /= scale;
    f.d /= scale;
    const int initial_sign = cccd_detail::sign(f, lower_bound);
    if(initial_sign == 0)
        return {lower_bound, true};

    Float critical[2];
    int   count = 0;
    if(f.a == 0)
    {
        // A quadratic is not necessarily monotone when its cubic term is zero.
        if(f.b != 0)
            critical[count++] = -0.5 * f.c / f.b;
    }
    else
    {
        const Float discriminant = std::fma(-3 * f.a, f.c, f.b * f.b);
        const Float error        = 32 * std::numeric_limits<Float>::epsilon()
                            * (f.b * f.b + std::abs(3 * f.a * f.c));
        if(std::abs(discriminant) <= error)
        {
            // Repeated derivative roots cannot be classified reliably here.
            return {lower_bound, true};
        }
        if(discriminant > 0)
        {
            const Float q = -(f.b + std::copysign(std::sqrt(discriminant), f.b));
            if(q == 0)
                return {lower_bound, true};
            critical[count++] = (q / f.a) / 3;
            critical[count++] = f.c / q;
            if(critical[0] > critical[1])
            {
                const Float swap = critical[0];
                critical[0]      = critical[1];
                critical[1]      = swap;
            }
        }
    }

    Float safe = lower_bound;
    for(int i = 0; i <= count; ++i)
    {
        const Float boundary = i == count ? 1 : critical[i];
        if(!(boundary > safe && boundary <= 1))
            continue;
        const int boundary_sign = cccd_detail::sign(f, boundary);
        if(boundary_sign != initial_sign)
        {
            // The supplementary algorithm may return an already advanced
            // critical point without further bisection.
            if(safe > lower_bound)
                return {safe, false};
            return {cccd_detail::safe_bisect(f, safe, boundary, initial_sign), false};
        }
        safe = boundary;
    }
    return {1, false};
}

inline UIPC_GENERIC Float point_triangle_relative_bound(const Vector3& dp,
                                                        const Vector3& dt0,
                                                        const Vector3& dt1,
                                                        const Vector3& dt2)
{
    return std::fmax((dp - dt0).norm(), std::fmax((dp - dt1).norm(), (dp - dt2).norm()));
}

inline UIPC_GENERIC Float edge_edge_relative_bound(const Vector3& da0,
                                                   const Vector3& da1,
                                                   const Vector3& db0,
                                                   const Vector3& db1)
{
    return std::fmax(std::fmax((da0 - db0).norm(), (da0 - db1).norm()),
                     std::fmax((da1 - db0).norm(), (da1 - db1).norm()));
}

// det(x1-x0, x2-x0, x3-x0) for linearly moving vertices. All differences
// remove common translation and velocity before forming the coefficients.
inline UIPC_GENERIC CubicPolynomial coplanarity_polynomial(const Vector3* x, const Vector3* dx)
{
    const Vector3 u = x[1] - x[0], v = x[2] - x[0], w = x[3] - x[0];
    const Vector3 du = dx[1] - dx[0], dv = dx[2] - dx[0], dw = dx[3] - dx[0];
    return {u.dot(v.cross(w)),
            du.dot(v.cross(w)) + u.dot(dv.cross(w)) + u.dot(v.cross(dw)),
            du.dot(dv.cross(w)) + du.dot(v.cross(dw)) + u.dot(dv.cross(dw)),
            du.dot(dv.cross(dw))};
}

namespace cccd_detail
{
    inline UIPC_GENERIC CCDStep pair_step(const Vector3* x,
                                          const Vector3* dx,
                                          Float          distance_squared,
                                          Float          speed,
                                          Float          thickness,
                                          Float          eta)
    {
        if(!std::isfinite(distance_squared) || distance_squared < 0
           || !std::isfinite(speed) || speed < 0 || !std::isfinite(thickness)
           || thickness < 0 || !std::isfinite(eta) || eta <= 0 || eta >= 1)
            return {};
        const Float distance = std::sqrt(distance_squared);
        if(distance <= thickness)
            return {};
        if(speed == 0)
            return {1, false};
        const Float lower = std::fmin(1.0, (1 - eta) * (distance - thickness) / speed);
        if(lower == 1)
            return {1, false};
        if(thickness > 0)
            return {lower, true};
        const CCDStep cubic = cubic_safe_step(coplanarity_polynomial(x, dx), lower);
        if(cubic.needs_fallback)
            return {lower, true};
        // Keep the distance-certified lower bound and shrink a root boundary
        // away from contact. A root-free full interval needs no contraction.
        return {cubic.fraction == 1 ? 1 : std::fmax(lower, (1 - eta) * cubic.fraction), false};
    }

    inline UIPC_GENERIC bool finite_pair(const Vector3* x, const Vector3* dx)
    {
        for(int i = 0; i < 4; ++i)
            if(!finite(x[i]) || !finite(dx[i]))
                return false;
        return true;
    }
}  // namespace cccd_detail

// Arrays are ordered [point, t0, t1, t2] and [a0, a1, b0, b1]. On fallback,
// fraction remains a conservative distance/relative-speed bound (possibly 0).
inline UIPC_GENERIC CCDStep point_triangle_safe_step(const Vector3* x,
                                                     const Vector3* dx,
                                                     Float          thickness,
                                                     Float          eta = 0.1)
{
    if(!cccd_detail::finite_pair(x, dx))
        return {};
    const Vector3 e0 = x[2] - x[1], e1 = x[3] - x[1];
    const Float   area2 = e0.cross(e1).squaredNorm();
    if(!std::isfinite(area2)
       || area2 <= 64 * std::numeric_limits<Float>::epsilon() * e0.squaredNorm()
                       * e1.squaredNorm())
        return {};
    const auto flag = distance::point_triangle_distance_flag(x[0], x[1], x[2], x[3]);
    Float squared;
    distance::point_triangle_distance2(flag, x[0], x[1], x[2], x[3], squared);
    return cccd_detail::pair_step(
        x, dx, squared, point_triangle_relative_bound(dx[0], dx[1], dx[2], dx[3]), thickness, eta);
}

inline UIPC_GENERIC CCDStep edge_edge_safe_step(const Vector3* x,
                                                const Vector3* dx,
                                                Float          thickness,
                                                Float          eta = 0.1)
{
    if(!cccd_detail::finite_pair(x, dx))
        return {};
    const auto flag = distance::edge_edge_distance_flag(x[0], x[1], x[2], x[3]);
    Float      squared;
    distance::edge_edge_distance2(flag, x[0], x[1], x[2], x[3], squared);
    return cccd_detail::pair_step(
        x, dx, squared, edge_edge_relative_bound(dx[0], dx[1], dx[2], dx[3]), thickness, eta);
}
}  // namespace uipc::backend::cuda::pncg
