import functools
import subprocess
import sys
import textwrap

import pytest


_LEAK_MARKERS = (
    "nanobind: leaked",
    "nanobind: this is likely caused by a reference counting issue",
)


def _run_in_subprocess(source: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", textwrap.dedent(source)],
        capture_output=True,
        check=False,
        text=True,
        timeout=15,
    )


@functools.cache
def _binding_backend() -> str:
    result = _run_in_subprocess(
        """
        from uipc._native.pyuipc import ResidentThread
        print(type(ResidentThread).__module__)
        """
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


@pytest.mark.basic
@pytest.mark.parametrize(
    ("name", "source"),
    [
        ("import", "import uipc"),
        (
            "resident_thread",
            """
            import threading

            from uipc._native.pyuipc import ResidentThread

            for _ in range(32):
                finished = threading.Event()
                worker = ResidentThread()
                assert worker.post(lambda: finished.set())
                assert finished.wait(5)
                del worker
            """,
        ),
        (
            "buffer_callbacks",
            """
            import gc
            import weakref

            from uipc._native.pyuipc.backend import Buffer, BufferView

            sizes = []
            buffer_view = BufferView(1234, 0, 4, 8, 8, "cpu")
            owner = {}

            def resize(size):
                sizes.append(size)
                owner["buffer"].last_size = size

            def get_buffer_view():
                return buffer_view

            buffer = Buffer(resize, get_buffer_view)
            owner["buffer"] = buffer
            buffer.label = "lifecycle-probe"
            buffer.resize(7)
            assert buffer.label == "lifecycle-probe"
            assert buffer.last_size == 7
            assert sizes == [7]
            assert buffer.view().handle() == 1234

            buffer_ref = weakref.ref(buffer)
            del buffer
            del owner["buffer"]
            gc.collect()
            assert buffer_ref() is None
            """,
        ),
        (
            "trampoline",
            """
            from uipc._native.pyuipc.core import PyIEngine

            class DummyEngine(PyIEngine):
                def __init__(self):
                    super().__init__()
                    self.frame = 0

                def do_init(self):
                    pass

                def do_advance(self):
                    self.frame += 1

                def do_sync(self):
                    pass

                def do_retrieve(self):
                    pass

                def do_to_json(self):
                    return {"frame": self.frame}

                def do_dump(self):
                    return True

                def do_recover(self, dst_frame):
                    self.frame = dst_frame
                    return True

                def get_frame(self):
                    return self.frame

            engine = DummyEngine()
            engine.do_advance()
            assert engine.get_frame() == 1
            assert engine.do_to_json() == {"frame": 1}
            assert not engine.status().has_error()
            """,
        ),
    ],
)
def test_binding_process_exits_cleanly(name: str, source: str) -> None:
    if name == "resident_thread" and _binding_backend() != "nanobind":
        pytest.skip("ResidentThread shutdown regression probe targets nanobind")

    result = _run_in_subprocess(source)
    assert result.returncode == 0, (
        f"{name} subprocess exited with {result.returncode}\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
    assert not any(marker in result.stderr for marker in _LEAK_MARKERS), (
        f"{name} subprocess reported a nanobind leak:\n{result.stderr}"
    )
