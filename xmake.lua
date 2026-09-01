set_xmakever("3.0.5")

-- Project-local package recipes that have not reached xmake-repo yet.
add_repositories("libuipc-packages xmake/repository", {rootdir = os.scriptdir()})

option("python_bindings", {default = false, description = "Build Python bindings"})
option("pybind", {
    default = false,
    description = "Deprecated alias for python_bindings"
})
option("python_binding")
    set_default("nanobind")
    set_values("nanobind")
    set_description("Deprecated compatibility option; nanobind is the only implementation")
    after_check(function (option)
        assert(option:value() == "nanobind",
            "python_binding no longer selects an implementation; only nanobind is supported")
    end)
option_end()
option("examples", {default = true})
option("tests", {default = true})
option("benchmarks", {default = false})
option("dev", {default = true, description = "Enable developer mode"})
option("github_actions", {default = false})

option("backend_cuda", {default = true, description = "Build with CUDA backend"})
option("usd", {default = false, description = "Build with OpenUSD support"})
option("vdb", {default = false, description = "Build with OpenVDB support"})

option("python_version", {default = "3.11.x", description = "Specify python version"})
option("python_system", {default = false, description = "Use system python"})


includes("external/GKlib", "external/METIS", "src", "apps", "xmake/*.lua")

add_rules("mode.release", "mode.debug", "mode.releasedbg", "uipc.basic")

set_languages("c++20")

if is_plat("linux") then
    add_rpathdirs("$ORIGIN")
end

set_version("0.9.0")

-- Repository policy forbids compiler-cache wrappers because they make build
-- provenance and CUDA diagnostics harder to reproduce.
set_policy("build.ccache", false)

if has_config("python_bindings") or has_config("pybind") then
    -- Python wheels must carry package runtimes even when matching libraries
    -- are installed on the build host.
    set_policy("install.strip_packagelibs", false)
end

if has_config("dev") then
    set_policy("compatibility.version", "3.0")

    if is_plat("windows") then
        set_runtimes("MD")
    end
end
