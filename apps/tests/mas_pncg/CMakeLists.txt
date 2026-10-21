# Keep this executable host-only when calling into the CUDA backend DLL.
uipc_add_test(mas_pncg scene.cpp)
target_link_libraries(mas_pncg PRIVATE uipc::backend::cuda)
