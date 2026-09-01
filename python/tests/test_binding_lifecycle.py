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
            "resident_thread_queued_shutdown",
            """
            import threading

            from uipc._native.pyuipc import ResidentThread

            started = threading.Event()
            release = threading.Event()
            finished = threading.Event()
            worker = ResidentThread()

            def first_callback():
                started.set()
                assert release.wait(5)

            assert worker.post(first_callback)
            assert started.wait(5)
            assert worker.post(lambda: finished.set())
            release.set()

            # Destruction joins the worker while the second Python callback is
            # queued. It must not deadlock by retaining the GIL during join.
            del worker
            assert finished.wait(5)
            """,
        ),
        (
            "resident_thread_interpreter_shutdown",
            """
            import atexit
            import os
            import threading

            from uipc._native.pyuipc import ResidentThread

            started = threading.Event()
            release = threading.Event()
            callback_ran = False
            worker = ResidentThread()

            def first_callback():
                started.set()
                assert release.wait(10)

            assert worker.post(first_callback)
            assert started.wait(5)

            def queued_callback():
                global callback_ran
                callback_ran = True

            assert worker.post(queued_callback)

            def destroy_worker():
                global worker
                del worker
                if callback_ran:
                    os._exit(23)

            # Standard atexit handlers run in reverse registration order:
            # release the active callback, then destroy the worker. The
            # binding's earlier threading shutdown hook must prevent the
            # queued callback from entering Python during interpreter exit.
            atexit.register(destroy_worker)
            atexit.register(release.set)
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
            import gc
            import tempfile

            from uipc import Engine, Scene, World
            from uipc._native.pyuipc.core import PyIEngine

            calls = []

            class DummyEngine(PyIEngine):
                def __init__(self):
                    super().__init__()
                    self.current_frame = 0

                def do_init(self):
                    calls.append("init")

                def do_advance(self):
                    calls.append("advance")
                    self.current_frame += 1

                def do_sync(self):
                    calls.append("sync")

                def do_retrieve(self):
                    calls.append("retrieve")

                def do_to_json(self):
                    return {"frame": self.current_frame}

                def do_dump(self):
                    calls.append("dump")
                    return True

                def do_recover(self, dst_frame):
                    calls.append("recover")
                    self.current_frame = dst_frame
                    return True

                def get_frame(self):
                    return self.current_frame

            implementation = DummyEngine()
            engine = Engine(
                "python-test",
                implementation,
                tempfile.mkdtemp(prefix="pyuipc-trampoline-"),
            )
            world = World(engine)
            scene = Scene()
            world.init(scene)

            # Drop Python's direct reference. Engine's C++ shared_ptr must keep
            # both the implementation and its Python overrides alive.
            del implementation
            gc.collect()

            world.advance()
            world.retrieve()
            assert calls == ["init", "advance", "sync", "retrieve"]
            assert engine.to_json() == {"frame": 1}
            assert world.frame() == 1
            assert world.dump()
            assert world.recover(7)
            assert world.frame() == 7

            del scene, world, engine
            gc.collect()
            """,
        ),
    ],
)
def test_binding_process_exits_cleanly(name: str, source: str) -> None:
    if name.startswith("resident_thread") and _binding_backend() != "nanobind":
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
