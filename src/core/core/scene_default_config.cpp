#include "scene_default_config.h"
#include <uipc/common/unit.h>

namespace uipc::core
{
geometry::AttributeCollection default_scene_config() noexcept
{
    geometry::AttributeCollection config;
    config.resize(1);
    config.create("dt", Float{0.01});
    config.create("gravity", Vector3{0.0, -9.8, 0.0});

    config.create("cfl/enable", IndexT{0});

    config.create("integrator/type", std::string{"bdf1"});

    config.create("newton/max_iter", IndexT{1024});
    config.create("newton/min_iter", IndexT{1});
    config.create("newton/use_adaptive_tol", IndexT{0});
    config.create("newton/velocity_tol", Float{0.05_m / 1.0_s});
    config.create("newton/ccd_tol", Float{1.0});
    config.create("newton/transrate_tol", Float{0.1 / 1.0_s});

    config.create("newton/semi_implicit/enable", IndexT{0});
    config.create("newton/semi_implicit/beta_tol", Float{1e-3});

    config.create("linear_system/tol_rate", Float{1e-3});

    // AGIPC coarse Newton solve defaults:
    // 1) tag protected/collapsible edges by Green-strain change,
    // 2) aggregate nodes into algebraic supernodes,
    // 3) solve the Galerkin coarse system, then
    // 4) refine on the original fine Hessian for a bounded number of PCG steps.
    // Disabled by default: AGIPC only accelerates FEM-heavy deformable scenes;
    // for rigid/ABD-dominated scenes it adds coarse-assembly overhead without DoF
    // reduction. Scenes that benefit must opt in via linear_system/coarse/enable=1.
    config.create("linear_system/coarse/enable", IndexT{0});
    config.create("linear_system/coarse/affine_threshold", IndexT{32});
    // 0 => adaptive fine-refinement budget scaled from coarse-PCG iterations
    // (reference behavior); a positive value forces a fixed number of fine PCG
    // correction iterations after the coarse solve.
    config.create("linear_system/coarse/fine_refine_iters", IndexT{0});
    config.create("linear_system/coarse/block_size", IndexT{256});
    config.create("linear_system/coarse/component_passes", IndexT{8});
    config.create("linear_system/coarse/max_aggregate_size", IndexT{32});
    config.create("linear_system/coarse/green_strain_tau", Float{5e-5});
    // The coarse solve is an inexact Newton-step correction that is followed by
    // bounded fine refinement, so it need not fully converge. Cap it low: most
    // well-conditioned coarse systems converge well under this bound, while
    // ill-conditioned (stiff-contact) frames would otherwise waste hundreds of
    // PCG iterations on a marginal residual gain.
    config.create("linear_system/coarse/max_iterations", IndexT{150});
    config.create("linear_system/coarse/pcg_relative_tol", Float{1e-3});
    config.create("linear_system/coarse/check_interval", IndexT{30});
    // Optional prolongated-displacement early stop (paper eps_d ~ 1e-3 * bbox
    // diagonal). 0 disables it; coarse termination then relies on the rz residual
    // tolerance and max_iterations. Scenes may set a scale-appropriate value.
    config.create("linear_system/coarse/displacement_tolerance", Float{0.0});
    config.create("linear_system/coarse/use_fine_preconditioner_after_coarse", IndexT{1});
    config.create("linear_system/coarse/use_mas_preconditioner", IndexT{0});

    // default:
    //  - fused_pcg
    // or:
    //  - linear_pcg (30% slower)
    config.create("linear_system/solver", std::string{"fused_pcg"});

    config.create("line_search/max_iter", IndexT{8});
    config.create("line_search/report_energy", IndexT{0});

    config.create("contact/enable", IndexT{1});
    config.create("contact/d_hat", Float{0.01});

    config.create("contact/friction/enable", IndexT{1});
    // friction transition velocity
    config.create("contact/eps_velocity", Float{0.01_m / 1.0_s});

    // default:
    //  - ipc
    // or:
    //  - al-ipc
    config.create("contact/constitution", std::string{"ipc"});

    // al-ipc tuning knobs. They are ignored when contact/constitution != "al-ipc".
    config.create("contact/al-ipc/mu_scale_fem", Float{5e7});
    config.create("contact/al-ipc/mu_scale_abd", Float{1e5});
    config.create("contact/al-ipc/toi_threshold", Float{0.1});
    config.create("contact/al-ipc/alpha_lower_bound", Float{1e-6});
    config.create("contact/al-ipc/decay_factor", Float{0.3});

    // adaptive contact tuning knobs.
    config.create("contact/adaptive/min_kappa", Float{100.0_MPa});
    config.create("contact/adaptive/init_kappa", Float{1.0_GPa});
    config.create("contact/adaptive/max_kappa", Float{100.0_GPa});


    // default:
    //  - info_stackless_bvh
    // or:
    //  - stackless_bvh
    //  - linear_bvh (slower)
    config.create("collision_detection/method", std::string{"info_stackless_bvh"});

    config.create("sanity_check/enable", IndexT{1});
    config.create("sanity_check/mode", std::string{"normal"});

    config.create("diff_sim/enable", IndexT{0});

    config.create("extras/debug/dump_surface", IndexT{0});
    config.create("extras/debug/dump_linear_system", IndexT{0});
    config.create("extras/debug/dump_linear_pcg", IndexT{0});
    config.create("extras/debug/dump_mas_matrices", IndexT{0});
    config.create("extras/strict_mode/enable", IndexT{0});

    return config;
}

static Json& nested_json(Json& j, const std::string_view path)
{
    size_t pos     = 0;
    Json*  current = &j;
    while(true)
    {
        size_t next_pos = path.find('/', pos);
        auto   key      = path.substr(pos, next_pos - pos);

        if(next_pos == std::string_view::npos)
        {
            return (*current)[key];
        }

        auto& child = (*current)[key];
        if(!child.is_object())
        {
            child = Json::object();
        }
        current = &child;
        pos     = next_pos + 1;
    }
}

Json to_config_json(const geometry::AttributeCollection& config)
{
    Json j;
    auto names = config.names();

    for(auto& name : names)
    {
        auto attr = config.find(name);
        UIPC_ASSERT_THROW(attr != nullptr, "Attribute '{}' not found in config.", name);
        auto& sub_json = nested_json(j, name);
        sub_json       = attr->to_json(0);
    }
    return j;
}

static const Json* find_nested_json(const Json& j, const std::string_view path)
{
    size_t      pos     = 0;
    const Json* current = &j;
    while(true)
    {
        size_t next_pos = path.find('/', pos);
        auto   key      = path.substr(pos, next_pos - pos);

        if(!current->is_object())
        {
            return nullptr;
        }

        auto it = current->find(key);
        if(it == current->end())
        {
            return nullptr;
        }

        if(next_pos == std::string_view::npos)
        {
            return &(*it);
        }

        current = &(*it);
        pos     = next_pos + 1;
    }
}

void from_config_json(geometry::AttributeCollection& config, const Json& j)
{
    auto names = config.names();
    for(auto& name : names)
    {
        auto attr = config.find(name);
        UIPC_ASSERT_THROW(attr != nullptr, "Attribute '{}' not found in config.", name);
        auto sub_json = find_nested_json(j, name);
        if(sub_json != nullptr)
        {
            // wrap it in an array to use from_json_array
            Json wrapper_array = Json::array();
            wrapper_array.push_back(*sub_json);
            attr->from_json_array(wrapper_array);
        }
    }
}
}  // namespace uipc::core
