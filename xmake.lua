set_xmakever("3.0.5")

option("pybind", {default = false, description = "Build pyuipc"})
option("examples", {default = true})
option("tests", {default = true})
option("benchmarks", {default = false})
option("dev", {default = true, description = "Enable developer mode"})
option("github_actions", {default = false})

option("backend_cuda", {default = true, description = "Build with CUDA backend"})
option("cuda_legacy_collision", {default = true, description = "Build legacy CUDA broad-phase trajectory filters"})
option("optix", {default = false, description = "Build the experimental OptiX Single-EF backend"})
option("optix_nvcc", {default = "", description = "Optional NVCC override for OptiX PTX compilation"})
option("usd", {default = false, description = "Build with OpenUSD support"})
option("vdb", {default = false, description = "Build with OpenVDB support"})

option("python_version", {default = "3.11.x", description = "Specify python version"})
option("python_system", {default = false, description = "Use system python"})

if has_config("optix") and not has_config("backend_cuda") then
    raise("optix requires backend_cuda")
end


includes("src", "apps", "xmake/*.lua")

add_rules("mode.release", "mode.debug", "mode.releasedbg", "uipc.basic")

set_languages("c++20")

if is_plat("linux") then
    add_rpathdirs("$ORIGIN")
end

set_version("0.9.0")

-- Repository policy forbids compiler-cache wrappers because they make build
-- provenance and CUDA diagnostics harder to reproduce.
set_policy("build.ccache", false)

if has_config("dev") then
    set_policy("compatibility.version", "3.0")

    if is_plat("windows") then
        set_runtimes("MD")
    end
end
