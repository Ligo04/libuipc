find_path(UIPC_OPTIX_INCLUDE_DIR optix.h
    HINTS "${UIPC_OPTIX_ROOT}/include" "$ENV{OptiX_ROOT}/include"
    REQUIRED)
if(NOT UIPC_OPTIX_NVCC)
    set(UIPC_OPTIX_NVCC "${CMAKE_CUDA_COMPILER}")
endif()
if(NOT EXISTS "${UIPC_OPTIX_NVCC}")
    message(FATAL_ERROR "OptiX PTX compiler does not exist: ${UIPC_OPTIX_NVCC}")
endif()

set(UIPC_OPTIX_DEVICE_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/collision_detection/optix/ef_device.cu")
set(UIPC_OPTIX_SHARED_HEADER
    "${CMAKE_CURRENT_SOURCE_DIR}/collision_detection/optix/ef_shared.h")
set(UIPC_OPTIX_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/optix")
set(UIPC_OPTIX_PTX "${UIPC_OPTIX_GENERATED_DIR}/ef_device.ptx")
set(UIPC_OPTIX_PTX_HEADER "${UIPC_OPTIX_GENERATED_DIR}/optix_ef_device_ptx.h")
set(UIPC_OPTIX_PTX_FLAGS --ptx --std=c++17 --gpu-architecture=compute_75)
if(CMAKE_CUDA_HOST_COMPILER)
    list(APPEND UIPC_OPTIX_PTX_FLAGS --compiler-bindir "${CMAKE_CUDA_HOST_COMPILER}")
endif()
file(GLOB_RECURSE UIPC_OPTIX_SDK_HEADERS CONFIGURE_DEPENDS
    "${UIPC_OPTIX_INCLUDE_DIR}/*.h")
add_custom_command(
    OUTPUT "${UIPC_OPTIX_PTX_HEADER}"
    BYPRODUCTS "${UIPC_OPTIX_PTX}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${UIPC_OPTIX_GENERATED_DIR}"
    COMMAND "${UIPC_OPTIX_NVCC}" ${UIPC_OPTIX_PTX_FLAGS}
        "-I${UIPC_OPTIX_INCLUDE_DIR}"
        "${UIPC_OPTIX_DEVICE_SOURCE}" -o "${UIPC_OPTIX_PTX}"
    COMMAND "${UIPC_PYTHON_EXECUTABLE_PATH}"
        "${PROJECT_SOURCE_DIR}/scripts/embed_optix_ptx.py"
        --input "${UIPC_OPTIX_PTX}" --output "${UIPC_OPTIX_PTX_HEADER}"
    DEPENDS "${UIPC_OPTIX_DEVICE_SOURCE}" "${UIPC_OPTIX_SHARED_HEADER}"
        "${PROJECT_SOURCE_DIR}/scripts/embed_optix_ptx.py"
        "${UIPC_OPTIX_NVCC}" ${UIPC_OPTIX_SDK_HEADERS}
    COMMENT "Compiling and embedding the OptiX Single-EF program"
    VERBATIM)
add_custom_target(uipc_optix_ptx DEPENDS "${UIPC_OPTIX_PTX_HEADER}")
add_dependencies(cuda_collision_objects uipc_optix_ptx)
target_include_directories(cuda_collision_objects PRIVATE
    "${UIPC_OPTIX_INCLUDE_DIR}" "${UIPC_OPTIX_GENERATED_DIR}")
find_package(CUDAToolkit REQUIRED)
target_link_libraries(cuda PRIVATE CUDA::cuda_driver ${CMAKE_DL_LIBS})
