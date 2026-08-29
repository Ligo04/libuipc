set(UIPC_NANOBIND_VERSION "3.0.0")

execute_process(
    COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} "-c"
            "from importlib.metadata import version; raise SystemExit(version('nanobind') != '${UIPC_NANOBIND_VERSION}')"
    RESULT_VARIABLE UIPC_NANOBIND_VERSION_RESULT
    OUTPUT_QUIET
)
if(NOT UIPC_NANOBIND_VERSION_RESULT EQUAL 0)
    uipc_info("Installing nanobind ${UIPC_NANOBIND_VERSION}...")
    uipc_require_pip_ensure(${UIPC_PYTHON_EXECUTABLE_PATH})
    execute_process(
        COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} "-m" "pip" "install"
                "nanobind==${UIPC_NANOBIND_VERSION}"
        RESULT_VARIABLE UIPC_NANOBIND_INSTALL_RESULT
    )
    if(NOT UIPC_NANOBIND_INSTALL_RESULT EQUAL 0)
        uipc_error("Python [${UIPC_PYTHON_EXECUTABLE_PATH}] failed to install nanobind ${UIPC_NANOBIND_VERSION}.")
    endif()
endif()

execute_process(
    COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH} "-m" "nanobind" "--cmake_dir"
    OUTPUT_VARIABLE nanobind_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE UIPC_NANOBIND_CMAKE_DIR_RESULT
)
if(NOT UIPC_NANOBIND_CMAKE_DIR_RESULT EQUAL 0)
    uipc_error("Failed to query the nanobind CMake package directory.")
endif()
file(TO_CMAKE_PATH "${nanobind_ROOT}" nanobind_ROOT)
uipc_info("nanobind CMake package directory: ${nanobind_ROOT}")

set(Python_EXECUTABLE ${UIPC_PYTHON_EXECUTABLE_PATH})
find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED QUIET)

# Some CMake/Python combinations report Python_SOABI with the platform module
# extension already appended. nanobind appends that extension itself, which
# would otherwise produce names such as `pyuipc.cpython-312-...so.so`.
execute_process(
    COMMAND "${Python_EXECUTABLE}" "-c"
            "import sysconfig; print(sysconfig.get_config_var('SOABI') or '')"
    OUTPUT_VARIABLE UIPC_PYTHON_SOABI
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE UIPC_PYTHON_SOABI_RESULT
)
if(NOT UIPC_PYTHON_SOABI_RESULT EQUAL 0)
    uipc_error("Failed to query SOABI from Python [${Python_EXECUTABLE}].")
endif()
set(Python_SOABI "${UIPC_PYTHON_SOABI}")
uipc_info("Python SOABI: ${Python_SOABI}")

find_package(nanobind ${UIPC_NANOBIND_VERSION} EXACT CONFIG REQUIRED QUIET)

add_subdirectory(pyuipc)
