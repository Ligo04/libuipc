#pragma once
#include <cmath>
#include <limits>

namespace uipc::backend::cuda
{
struct PNCGSubspaceResult
{
    double mu           = 0;
    double nu           = 0;
    bool   used_history = false;
    bool   valid        = false;
};

inline PNCGSubspaceResult solve_pncg_subspace(double a, double b, double c, double gz, double gp, bool history)
{
    PNCGSubspaceResult result;
    if(!std::isfinite(a) || !std::isfinite(gz) || a <= 0 || gz <= 0)
        return result;
    result.mu    = gz / a;
    result.valid = std::isfinite(result.mu) && result.mu > 0;
    if(!result.valid || !history || !std::isfinite(b) || !std::isfinite(c)
       || !std::isfinite(gp) || c <= 0)
        return result;
    const double correlation = b / std::sqrt(a) / std::sqrt(c);
    const double schur       = c * (1.0 - correlation * correlation);
    if(!std::isfinite(schur) || schur <= 64 * std::numeric_limits<double>::epsilon() * c)
        return result;
    const double nu    = (b / a * gz - gp) / schur;
    const double mu    = gz / a + (b / a) * nu;
    const double slope = -mu * gz + nu * gp;
    if(std::isfinite(mu) && std::isfinite(nu) && std::isfinite(slope) && slope < 0)
    {
        result.mu           = mu;
        result.nu           = nu;
        result.used_history = true;
    }
    return result;
}
}  // namespace uipc::backend::cuda
