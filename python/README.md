# PYUIPC

the python binding for libuipc

## Build


### uv build

Build the wheel manually (assumes native libraries are already built):

```bash 
uv sync --no-dev
uv build
```

## Dev

```bash 
uv sync
```

### CMake presets (recommended)

Build the C++ bindings for local development:

```bash
cmake --preset ci-build-wheel
cmake --build --preset ci-build-wheel
```

Set the Python interpreter explicitly if needed:

```bash
cmake --preset ci-build-wheel -DUIPC_PYTHON_EXECUTABLE_PATH=/path/to/python
cmake --build --preset ci-build-wheel
```

### xmake pack (recommended)

Build the wheel with xmake, which handles native compilation, dependency bundling, stub generation, and versioning automatically:

```bash
xmake pack pyuipc
```

The `.whl` file will be output to `build/xpack/pyuipc/`.

### Editable install

Install the Python package in editable mode (will build native code via CMake):

```bash
pip install -e ./python
```
