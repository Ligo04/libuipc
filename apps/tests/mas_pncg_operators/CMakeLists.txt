# Compile one copy of the provider device code and avoid linking the CUDA backend DLL.
uipc_add_test(mas_pncg_operators
    preconditioner.cu
    fused.cu
    mollified_ee.cu
    conservative_ccd.cu
    local_ccd.cu
    ../backends/cuda/mas_pncg_math.cu
    ${PROJECT_SOURCE_DIR}/src/backends/cuda/finite_element/mas_preconditioner_engine.cu)
target_include_directories(mas_pncg_operators PRIVATE
    "${PROJECT_SOURCE_DIR}/src"
    "${PROJECT_SOURCE_DIR}/src/backends/cuda"
    "${PROJECT_SOURCE_DIR}/src/backends/cuda/cuda_tool")
if(CUDAToolkit_VERSION_MAJOR GREATER_EQUAL 13)
    target_include_directories(mas_pncg_operators PRIVATE ${CUDAToolkit_INCLUDE_DIRS}/cccl)
endif()
target_link_libraries(mas_pncg_operators PRIVATE CUDA::cudart)
target_compile_options(mas_pncg_operators PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
    $<$<COMPILE_LANGUAGE:CUDA>:--diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028,20208>)
if(MSVC AND CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 13.0)
    target_compile_options(mas_pncg_operators PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
endif()
set_target_properties(mas_pncg_operators PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20)
uipc_set_target_cuda_architectures(mas_pncg_operators "${UIPC_CUDA_ARCHITECTURES}")
