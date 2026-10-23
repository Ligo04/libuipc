#include <catch2/catch_all.hpp>
#include <uipc/uipc.h>
#include <algorithm>
#include <limits>
#include <set>

TEST_CASE("scene_config_schema_and_validation", "[scene][config]")
{
    using namespace uipc;
    using namespace uipc::core;

    const auto  schema  = Scene::config_schema();
    const auto& entries = schema.at("entries");

    REQUIRE(schema.at("schemaVersion") == 1);
    REQUIRE(schema.at("strictUnknownKeys") == true);
    std::set<std::string> schema_keys;
    std::set<std::string> default_keys;
    auto flatten_defaults = [&](auto&& self, const Json& config, const std::string& prefix) -> void
    {
        for(auto&& [name, value] : config.items())
        {
            const auto path = prefix.empty() ? name : prefix + "/" + name;
            if(value.is_object())
                self(self, value, path);
            else
            {
                CAPTURE(path);
                default_keys.emplace(path);
                REQUIRE(entries.contains(path));
                REQUIRE(entries.at(path).at("default") == value);
            }
        }
    };
    flatten_defaults(flatten_defaults, Scene::default_config(), "");
    for(auto&& [name, entry] : entries.items())
        schema_keys.emplace(name);
    REQUIRE(schema_keys == default_keys);
    REQUIRE(entries.at("nonlinear_system/solver").at("default") == "newton");
    REQUIRE(entries.at("nonlinear_system/solver").at("enum")
            == Json::array({"newton", "mas_pncg"}));
    REQUIRE(entries.at("nonlinear_system/mas_pncg/ccd_mode").at("const") == "global");
    REQUIRE(entries.at("nonlinear_system/mas_pncg/iteration_policy").at("default") == "strict");
    REQUIRE(entries.at("nonlinear_system/mas_pncg/iteration_policy").at("enum")
            == Json::array({"strict", "paper"}));
    const auto& preconditioned_tol =
        entries.at("nonlinear_system/mas_pncg/preconditioned_gradient_tol");
    REQUIRE(preconditioned_tol.at("default").get<Float>() == Catch::Approx(1.0e-5));
    REQUIRE(preconditioned_tol.at("finite") == true);
    REQUIRE(preconditioned_tol.at("exclusiveMinimum") == 0.0);
    REQUIRE(preconditioned_tol.at("unit") == "m");
    REQUIRE_FALSE(preconditioned_tol.contains("maximum"));
    REQUIRE_FALSE(preconditioned_tol.contains("exclusiveMaximum"));
    REQUIRE(entries.at("dt").at("default").get<Float>() == Catch::Approx(0.01));
    REQUIRE(entries.at("dt").at("unit") == "s");
    REQUIRE(entries.at("gravity").at("componentCount") == 3);
    REQUIRE(entries.at("newton/semi_implicit/enable").at("default").get<IndexT>() == 1);
    REQUIRE(entries.at("newton/semi_implicit/K_min").at("default").get<IndexT>() == 6);
    REQUIRE(entries.at("newton/use_adaptive_tol").at("status") == "reserved");
    REQUIRE(entries.at("newton/use_adaptive_tol").at("const") == 0);
    REQUIRE(entries.at("sanity_check/mode").at("enum") == Json::array({"normal", "quiet"}));
    REQUIRE(entries.at("contact/al-ipc/mu_scale_mode").at("default") == "per_vertex");
    REQUIRE(entries.at("contact/al-ipc/mu_scale_mode").at("enum")
            == Json::array({"diag_norm", "per_vertex"}));
    REQUIRE(entries.at("contact/al-ipc/mu_scale_diag_norm").at("default").get<Float>()
            == Catch::Approx(0.1));

    const auto& collision          = entries.at("collision_detection/method");
    const auto& methods            = collision.at("enum");
    const auto& conditional_values = collision.at("conditionalValues");
    const bool legacy_collision_built = conditional_values.at("enabled").get<bool>();
    REQUIRE(methods.front() == "info_stackless_bvh");
    REQUIRE(conditional_values.at("backend") == "cuda");
    REQUIRE(conditional_values.at("buildOption") == "UIPC_WITH_CUDA_LEGACY_COLLISION");
    for(const auto& legacy_method : conditional_values.at("values"))
    {
        const bool listed = std::ranges::find(methods, legacy_method) != methods.end();
        REQUIRE(listed == legacy_collision_built);
    }

    for(auto&& [name, entry] : entries.items())
    {
        CAPTURE(name);
        REQUIRE(entry.contains("default"));
        REQUIRE(entry.contains("type"));
        REQUIRE(entry.contains("storageType"));
        REQUIRE(entry.contains("description"));
        REQUIRE(entry.contains("consumers"));
        REQUIRE(entry.contains("lifecycle"));
        REQUIRE(entry.contains("status"));
    }

    SECTION("constructor constraints")
    {
        auto config  = Scene::default_config();
        config["dt"] = 0.0;
        REQUIRE_THROWS(Scene{config});

        config                       = Scene::default_config();
        config["integrator"]["type"] = "rk4";
        REQUIRE_THROWS(Scene{config});

        config                               = Scene::default_config();
        config["newton"]["use_adaptive_tol"] = 1;
        REQUIRE_THROWS(Scene{config});

        config                                       = Scene::default_config();
        config["contact"]["al-ipc"]["mu_scale_mode"] = "unknown";
        REQUIRE_THROWS(Scene{config});

        config                                  = Scene::default_config();
        config["collision_detection"]["method"] = "linear_bvh";
        if(legacy_collision_built)
            REQUIRE_NOTHROW(Scene{config});
        else
            REQUIRE_THROWS(Scene{config});

        config                                      = Scene::default_config();
        config["contact"]["adaptive"]["min_kappa"]  = 2.0e9;
        config["contact"]["adaptive"]["init_kappa"] = 1.0e9;
        REQUIRE_THROWS(Scene{config});
    }

    SECTION("mutable config is revalidated")
    {
        Scene scene;
        auto  dt = scene.config().find<Float>("dt");
        REQUIRE(dt);
        view(*dt)[0] = -1.0;
        REQUIRE_THROWS(scene.validate_config());
    }

    SECTION("MAS-PNCG is selected through config")
    {
        auto config                          = Scene::default_config();
        config["nonlinear_system"]["solver"] = "mas_pncg";
        REQUIRE_NOTHROW(Scene{config});
        config["integrator"]["type"] = "bdf2";
        REQUIRE_THROWS(Scene{config});
        config["integrator"]["type"]      = "bdf1";
        config["contact"]["constitution"] = "al-ipc";
        REQUIRE_THROWS(Scene{config});
        config["contact"]["constitution"] = "ipc";
        config["diff_sim"]["enable"]      = 1;
        REQUIRE_THROWS(Scene{config});
        config["diff_sim"]["enable"]                    = 0;
        config["nonlinear_system"]["mas_pncg"]["top_k"] = 9;
        REQUIRE_THROWS(Scene{config});
        config["nonlinear_system"]["mas_pncg"]["top_k"]    = 8;
        config["nonlinear_system"]["mas_pncg"]["ccd_mode"] = "localized";
        REQUIRE_THROWS(Scene{config});
    }

    SECTION("MAS-PNCG policies preserve the linear solver selection")
    {
        for(auto policy : {"strict", "paper"})
            for(auto linear_solver : {"fused_pcg", "linear_pcg"})
            {
                CAPTURE(policy, linear_solver);
                auto config                          = Scene::default_config();
                config["nonlinear_system"]["solver"] = "mas_pncg";
                config["nonlinear_system"]["mas_pncg"]["iteration_policy"] = policy;
                config["linear_system"]["solver"] = linear_solver;
                Scene scene{config};
                REQUIRE(
                    scene.config().find<std::string>("linear_system/solver")->view()[0]
                    == linear_solver);
                REQUIRE_NOTHROW(scene.validate_config());
            }
    }

    SECTION("MAS-PNCG policy validates its enum and numerical bounds")
    {
        auto config                          = Scene::default_config();
        config["nonlinear_system"]["solver"] = "mas_pncg";
        auto& pncg               = config["nonlinear_system"]["mas_pncg"];
        pncg["iteration_policy"] = "unknown";
        REQUIRE_THROWS(Scene{config});
        pncg["iteration_policy"] = "paper";
        for(auto value : {0.0, -1.0})
        {
            CAPTURE(value);
            pncg["preconditioned_gradient_tol"] = value;
            REQUIRE_THROWS(Scene{config});
        }
        for(auto value : {1.0e-12, 1.0e-5, 1.0, 2.0})
        {
            pncg["preconditioned_gradient_tol"] = value;
            REQUIRE_NOTHROW(Scene{config});
        }
        for(auto value : {std::numeric_limits<Float>::quiet_NaN(),
                          std::numeric_limits<Float>::infinity(),
                          -std::numeric_limits<Float>::infinity()})
        {
            CAPTURE(value);
            Scene scene{config};
            auto  attribute = scene.config().find<Float>(
                "nonlinear_system/mas_pncg/preconditioned_gradient_tol");
            REQUIRE(attribute);
            view(*attribute)[0] = value;
            REQUIRE_THROWS(scene.validate_config());
        }
    }
}

TEST_CASE("tabular_extension_configs_are_not_silently_ignored", "[scene][config]")
{
    using namespace uipc;
    using namespace uipc::core;

    Scene scene;

    auto& contact   = scene.contact_tabular();
    auto  contact_a = contact.create("a");
    auto  contact_b = contact.create("b");
    REQUIRE_THROWS(
        contact.insert(contact_a, contact_b, 0.5, 1.0e8, true, Json{{"ignored", 1}}));
    REQUIRE_THROWS(contact.default_model(0.5, 1.0e8, true, Json{{"ignored", 1}}));

    auto& subscene   = scene.subscene_tabular();
    auto  subscene_a = subscene.create("a");
    auto  subscene_b = subscene.create("b");
    REQUIRE_THROWS(subscene.insert(subscene_a, subscene_b, true, Json{{"ignored", 1}}));
}
