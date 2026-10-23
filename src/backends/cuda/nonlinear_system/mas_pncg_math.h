#pragma once
#include <cmath>
#include <limits>

namespace uipc::backend::cuda
{
enum class PNCGEnergyPolicy
{
    StrictArmijo,
    PaperFinite,
    PaperMonotone
};

#ifdef __CUDACC__
__host__ __device__
#endif
    inline bool
    accepts_pncg_energy(double initial, double trial, double fraction, double slope, PNCGEnergyPolicy policy)
{
    if(!std::isfinite(initial) || !std::isfinite(trial)
       || !std::isfinite(fraction) || fraction <= 0)
        return false;
    if(policy == PNCGEnergyPolicy::StrictArmijo)
        return trial <= initial + 1e-4 * fraction * slope;
    if(fraction > 1)
        return false;
    if(policy == PNCGEnergyPolicy::PaperFinite)
        return true;
    // Optional engineering safeguard. Scale roundoff only by the actual
    // energies: a unit-valued floor would change behavior with energy units.
    const double allowance = 32 * std::numeric_limits<double>::epsilon()
                             * std::fmax(std::abs(initial), std::abs(trial));
    return trial <= initial + allowance;
}

struct PNCGSubspaceResult
{
    double mu           = 0;
    double nu           = 0;
    bool   used_history = false;
    bool   valid        = false;
};

#ifdef __CUDACC__
__host__ __device__
#endif
    inline PNCGSubspaceResult
    solve_pncg_subspace(double a, double b, double c, double gz, double gp, bool history)
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
