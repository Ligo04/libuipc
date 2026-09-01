includes("@builtin/xpack")

xpack("pyuipc")
    set_formats("zip")
    set_extension(".whl")

    add_targets("pyuipc")

    on_package(function (package)
        import("target.action.install", {alias = "_do_install_target"})
        import("lib.detect.find_tool")
        import("core.project.config")
        import("core.base.semver")
        import("helper")
        -- Copy project file
        local build_dir = path.absolute(package:builddir())
        os.tryrm(build_dir)
        os.mkdir(build_dir)
        os.vcp(path.join(os.projectdir(), "python/*"), build_dir)

        local pyuipc_target = package:target("pyuipc")
        local modules_dir = path.join(build_dir, "src/uipc/_native")
        os.mkdir(modules_dir)
        -- Copy project shared libraries
        -- A wheel must carry package runtimes even when matching libraries are
        -- installed on the build host. XMake's default strip policy resolves
        -- against host libraries and can otherwise omit runtimes such as TBB.
        _do_install_target(pyuipc_target, {
            headers = false,
            binaries = false,
            libraries = true,
            packages = true,
            installdir = modules_dir,
            libdir = "",
            bindir = "",
        })
        os.rm(path.join(modules_dir, "*.lib"))

        -- Build stub file
        local python = helper.get_python_program(pyuipc_target)

        local ok = try { function()
            os.vrunv(python, {
                "-c",
                "from importlib.metadata import version; " ..
                "assert version('nanobind') == '3.0.0'; import numpy"
            })
            return true
        end }

        local uv = assert(find_tool("uv"), "uv not found!")
        if is_plat("linux") then
            -- Every ELF copied into the wheel must resolve transitive package
            -- dependencies from its own directory. In particular,
            -- libtbbmalloc_proxy depends on the sibling libtbbmalloc library.
            for _, shared_library in ipairs(os.files(path.join(modules_dir, "*.so*"))) do
                if not os.islink(shared_library) then
                    os.vrunv(uv.program, {
                        "tool", "run",
                        "--from", "patchelf==0.17.2.4",
                        "patchelf",
                        "--set-rpath", "$ORIGIN",
                        shared_library,
                    })
                end
            end
        end

        if not ok then
            local requirements = {"nanobind==3.0.0", "numpy"}
            os.vrunv(uv.program,
                table.join({"pip", "install", "--python", python}, requirements))
        end

        local LD_LIBRARY_PATH = path.splitenv(os.getenv("LD_LIBRARY_PATH") or "")
        table.insert(LD_LIBRARY_PATH, 1, modules_dir)
        local python_libdir = try { function()
            local out = os.iorunv(python, {"-c", "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"})
            return out:trim()
        end }
        if python_libdir then
            table.insert(LD_LIBRARY_PATH, python_libdir)
        end
        os.vrunv(python, {
            path.join(os.projectdir(), "scripts/stubgen.py"),
            "--source-dir=" .. path.join(build_dir, "src"),
            "--output-dir=" .. path.join(build_dir, "src/uipc/_native"),
            "--marker-file=" .. path.join(build_dir, "src/uipc/py.typed"),
        }, {setenvs = {["LD_LIBRARY_PATH"] = path.joinenv(LD_LIBRARY_PATH)}})

        -- Build .whl file
        local cuda_version = helper.get_cuda_version()
        local python_version = semver.new(helper.get_python_version(pyuipc_target))
        local git_rev = try { function()
            return os.iorunv("git", {"rev-parse", "--short", "HEAD"})
        end }
        if git_rev then
            git_rev = git_rev:trim()
        end

        local argv = {
            "build",
            "--wheel", -- don't package tarball
            "--python", format("%s.%s", python_version:major(), python_version:minor()),
            "--out-dir", path.absolute(package:outputdir())
        }

        local opt = {
            curdir = build_dir,
            envs = {
                CUDA_VERSION = helper.format_version_with_prefix("cu", cuda_version),
                BUILD_MODE = config.get("mode"),
                GIT_REV = git_rev,
            },
        }

        os.vrunv(uv.program, argv, opt)
    end)
