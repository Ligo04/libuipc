target("backend_cuda")
    set_group("tests")
    add_rules("uipc_test")
    add_deps("cuda")
    -- Share the backend's kernel registry when inline CUDA helpers cross the DSO boundary.
    add_links("cudart")
    add_files("**.cu", "**.cpp")
    add_includedirs(path.join(os.projectdir(), "src"))
    if has_config("dev") then
        add_rules("clangd")
    end
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_cuflags("/wd4819", {tools = "cl"})
    -- parity with apps/tests/backends/cuda/CMakeLists.txt: the test .cu files
    -- use cuda_tool launch/dense math (needs extended lambdas + relaxed
    -- constexpr), and CUDA >= 13 CCCL requires the conforming MSVC preprocessor.
    add_cuflags("--extended-lambda", "--expt-relaxed-constexpr")
    if is_plat("windows") then
        add_cuflags("-Xcompiler=/Zc:preprocessor")
    end
