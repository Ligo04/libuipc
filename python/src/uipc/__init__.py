import os
import pathlib

from ._native import pyuipc


def init():
    if pyuipc.__file__ is None:
        err_message = """Python binding is not built.
        Please make a `Release` or `RelWithDebInfo` build with option `-DUIPC_BUILD_PYTHON_BINDINGS=1` to enable the Python binding."""
        raise Exception(err_message)

    # get module path
    module_path = pathlib.Path(pyuipc.__file__).absolute()
    module_dir = module_path.parent

    config = pyuipc.default_config()
    config["module_dir"] = str(module_dir)
    pyuipc.init(config)


if os.environ.get("NB_STUBGEN") != "1":
    init()


def _install_keyword_aliases():
    """Expose Python-safe spellings for native names that are keywords."""
    engine_status = pyuipc.core.EngineStatus
    engine_status_type = engine_status.Type
    legacy_none = getattr(engine_status_type, "None")
    if not hasattr(engine_status_type, "None_"):
        setattr(engine_status_type, "None_", legacy_none)
    if not hasattr(engine_status, "None_"):
        setattr(engine_status, "None_", legacy_none)


_install_keyword_aliases()

# import all pyuipc modules
from ._native.pyuipc import *  # noqa: E402,F403

__version__ = pyuipc.__version__
