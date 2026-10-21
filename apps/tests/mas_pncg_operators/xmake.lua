target("mas_pncg_operators")
    set_group("tests")
    add_rules("uipc_test", "cuda_warning", "cuda_no_host_compiler_check")
    -- Compile one copy of the provider device code without the CUDA backend DLL.
    add_files("preconditioner.cu", "../backends/cuda/mas_pncg_math.cu")
    add_files(path.join(os.projectdir(), "src/backends/cuda/finite_element/mas_preconditioner_engine.cu"))
    add_includedirs(path.join(os.projectdir(), "src"),
                    path.join(os.projectdir(), "src/backends/cuda"),
                    path.join(os.projectdir(), "src/backends/cuda/cuda_tool"))
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr", "-rdc=true")
    add_links("cudart")
    on_load(function(target)
        if target:is_plat("windows") then
            target:add("defines", "__NV_NO_HOST_COMPILER_CHECK")
            target:add("cuflags", "-allow-unsupported-compiler", "-Xcompiler=/wd4819", "-Xcompiler=/Zc:preprocessor")
        end
        target:set("toolchains", "cuda")
    end)
