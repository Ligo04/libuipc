#include <pyuipc/common/resident_thread.h>
#include <uipc/common/resident_thread.h>

#include <atomic>
#include <memory>

namespace pyuipc
{
using namespace uipc;

namespace
{
    std::atomic_bool python_shutting_down{false};

    class PythonResidentThread
    {
      public:
        PythonResidentThread()
            : m_thread{std::make_unique<ResidentThread>()}
        {
        }

        ~PythonResidentThread()
        {
            if(!m_thread)
                return;

            // ResidentThread drains an already queued task before joining. Python
            // destroys bound instances while holding the GIL, so release it while
            // the worker finishes a callback that may need to acquire it.
            if(py::is_alive() && PyGILState_Check())
            {
                py::gil_scoped_release release;
                m_thread.reset();
            }
            else
            {
                m_thread.reset();
            }
        }

        ResidentThread& thread() { return *m_thread; }

      private:
        std::unique_ptr<ResidentThread> m_thread;
    };
}  // namespace

PyResidentThread::PyResidentThread(py::module_& m)
{
    python_shutting_down.store(false, std::memory_order_release);

    // CPython invokes threading's early-exit callbacks before regular atexit
    // handlers and before runtime finalization. Mark the boundary while it is
    // still safe for an already running callback to release Python resources;
    // callbacks that are merely queued will then skip Python entirely.
    py::module_ threading = py::module_::import_("threading");
    threading.attr("_register_atexit")(py::cpp_function(
        [] { python_shutting_down.store(true, std::memory_order_release); }));

    auto class_ResidentThread = py::class_<PythonResidentThread>(
        m, "ResidentThread", R"(Resident thread for executing Python callables in a separate thread.)");
    class_ResidentThread.def(py::init<>(), R"(Create a new resident thread.)");
    class_ResidentThread.def(
        "post",
        [](PythonResidentThread& self, py::callable func)
        {
            // func must be a callable object with no arguments
            PYUIPC_ASSERT(py::isinstance<py::callable>(func), "func must be a callable object");
            return self.thread().post(
                [func = std::move(func)]() mutable
                {
                    // A queued callback may outlive the Python interpreter.
                    // Abandon the reference when Python can no longer be
                    // entered; releasing it avoids an unsafe DECREF later on
                    // the worker thread.
                    if(python_shutting_down.load(std::memory_order_acquire)
                       || !py::is_alive())
                    {
                        func.release();
                        return;
                    }

                    py::gil_scoped_acquire acquire;
                    if(!acquire.is_valid())
                    {
                        func.release();
                        return;
                    }

                    try
                    {
                        func();
                    }
                    catch(const std::exception& e)
                    {
                        logger::error("Exception in ResidentThread:", e.what());
                    }

                    // ResidentThread destroys the std::function on its worker
                    // thread after this callable returns. Release the Python
                    // reference while the GIL is still held so that the later
                    // C++ destruction is a no-op.
                    func.reset();
                });
        },
        py::arg("func"),
        R"(Post a callable to be executed in the resident thread.
Args:
    func: Python callable object (function) with no arguments to execute.
Returns:
    bool: True if the function was successfully posted.)");
    class_ResidentThread.def(
        "is_ready",
        [](PythonResidentThread& self) { return self.thread().is_ready(); },
        R"(Check if the resident thread is ready to accept new tasks.
Returns:
    bool: True if the thread is ready, False otherwise.)");
}
}  // namespace pyuipc
