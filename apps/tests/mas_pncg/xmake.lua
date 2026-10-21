target("mas_pncg")
    set_group("tests")
    add_rules("uipc_test")
    add_deps("cuda")
    -- Keep this executable host-only when calling into the CUDA backend DLL.
    add_files("scene.cpp")
