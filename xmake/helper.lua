-- @param prefix (string) "cp", "cu"
-- @param version_str (string) "3.10" "12.4"
-- @return (string) "cp310", "cu124"
function format_version_with_prefix(prefix, version_str)
    if type(prefix) ~= "string" or type(version_str) ~= "string" then
        return nil, "Invalid input: prefix and version_str must be strings."
    end

    local number_part = string.gsub(version_str, "%.", "")
    
    return prefix .. number_part
end

function get_cuda_version()
    import("core.base.option")
    import("core.project.config")
    import("lib.detect.find_programver")
    import("detect.sdks.find_cuda")

    local cuda_version = option.get("cuda_version")
    if not cuda_version then
        local cuda = assert(find_cuda(config.get("cuda")))
        cuda_version = find_programver(path.join(cuda.bindir, "nvcc"), {parse = "release (%d+%.%d+),"})
    end
    return cuda_version
end

function get_python_program(target)
    import("lib.detect.find_tool")

    local envs = target:pkgenvs()
    local python = find_tool("python3", {envs = envs})
    if not python then
        python = find_tool("python", {envs = envs})
    end
    python = assert(python, "python not found!")

    -- find_tool may return an executable command without its platform suffix
    -- (for example, python3 on Windows). External tools such as uv require the
    -- canonical interpreter path, so ask the selected Python for it directly.
    local program = os.iorunv(python.program, {
        "-c", "import sys; print(sys.executable)",
    }, {envs = envs}):trim()
    return assert(program ~= "" and program, "python executable path not found!")
end

function get_python_version(target)
    import("lib.detect.find_programver")

    return find_programver(get_python_program(target))
end
