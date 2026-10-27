rule("uipc.optix.ptx")
    on_load(function (target)
        target:add("includedirs", path.join(target:autogendir(), "optix"))
    end)
    before_build(function (target)
        import("core.project.config")
        import("core.project.depend")
        import("core.tool.compiler")
        import("lib.detect.find_tool")

        local optix = assert(target:pkg("optix"), "OptiX SDK not found")
        local includedirs = table.join(optix:get("includedirs") or {}, optix:get("sysincludedirs") or {})
        local include_dir = assert(includedirs[1], "OptiX SDK include directory not found")
        local compinst = compiler.load("cu", {target = target})
        local nvcc = config.get("optix_nvcc")
        if not nvcc or nvcc == "" then
            nvcc = compinst:program()
        end
        local python = assert(find_tool("python3", {envs = target:pkgenvs()})
            or find_tool("python", {envs = target:pkgenvs()}), "Python not found")
        local source = path.join(os.projectdir(), "src/backends/cuda/collision_detection/optix/ef_device.cu")
        local shared = path.join(path.directory(source), "ef_shared.h")
        local helper = path.join(os.projectdir(), "scripts/embed_optix_ptx.py")
        local generated = path.join(target:autogendir(), "optix")
        local ptx = path.join(generated, "ef_device.ptx")
        local header = path.join(generated, "optix_ef_device_ptx.h")
        local args = {"--ptx", "--std=c++17", "--gpu-architecture=compute_75",
            "-I" .. include_dir, source, "-o", ptx}
        local inputs = table.join({source, shared, helper, nvcc}, os.files(path.join(include_dir, "**.h")))
        depend.on_changed(function ()
            os.mkdir(generated)
            os.vrunv(nvcc, args, {envs = compinst:runenvs()})
            os.vrunv(python.program, {helper, "--input", ptx, "--output", header},
                {envs = target:pkgenvs()})
        end, {dependfile = target:dependfile(header), files = inputs,
            values = {nvcc, args, python.program},
            changed = not os.isfile(header) or not os.isfile(ptx)})
    end)
