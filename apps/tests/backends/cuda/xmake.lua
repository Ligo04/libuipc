target("backend_cuda")
    set_group("tests")
    add_rules("uipc_test")
    add_deps("cuda")
    add_files("**.cu", "**.cpp")
    -- These kernels run in a standalone executable without the backend DLL.
    remove_files("mas_pncg_math.cu")
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
