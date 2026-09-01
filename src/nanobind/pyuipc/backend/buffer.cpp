#include <pyuipc/backend/buffer.h>
#include <uipc/backend/buffer.h>
#include <uipc/common/log.h>
#include <utility>

namespace pyuipc::backend
{
using namespace uipc::backend;

namespace
{
    constexpr const char* BufferCallbacks = "_uipc_callbacks";
}

PyBuffer::PyBuffer(py::module_& m)
{
    // allow add attributes to this class
    auto class_Buffer =
        py::class_<Buffer>(m, "Buffer", py::dynamic_attr(), py::is_weak_referenceable());

    class_Buffer.def(
        "__init__",
        [](py::pointer_and_handle<Buffer> self, py::callable resize_func, py::callable get_buffer_view_func)
        {
            self.h.attr(BufferCallbacks) =
                py::make_tuple(std::move(resize_func), std::move(get_buffer_view_func));
            py::weakref owner{self.h};

            new(self.p)
                Buffer{[owner](SizeT size)
                       {
                           py::gil_scoped_acquire acquire;
                           py::object             instance = owner();
                           if(instance.is_none())
                               return;

                           py::tuple callbacks =
                               py::cast<py::tuple>(instance.attr(BufferCallbacks));
                           py::cast<py::callable>(callbacks[0])(size);
                       },
                       [owner]() -> BufferView
                       {
                           py::gil_scoped_acquire acquire;
                           py::object             instance = owner();
                           if(instance.is_none())
                               return {};

                           py::tuple callbacks =
                               py::cast<py::tuple>(instance.attr(BufferCallbacks));
                           return py::cast<BufferView>(
                               py::cast<py::callable>(callbacks[1])());
                       }};
        },
        py::arg("resize_func"),
        py::arg("get_buffer_view_func"),
        R"(Constructs a Buffer object with provided resize and get_buffer_view functions.
Args:
    resize_func f:(int)->None: Function to resize the buffer.
    get_buffer_view_func f:()->BufferView: Function to retrieve the buffer view.)");

    class_Buffer.def(
        "resize",
        [](Buffer& self, SizeT size)
        {
            try
            {
                self.resize(size);
            }
            catch(const std::exception& e)
            {
                logger::error(PYUIPC_MSG("Error in resize_func: {}", e.what()));
            }
        },
        py::arg("size"),
        R"(Resize the buffer.
Args:
    size: New size in elements.)");

    class_Buffer.def(
        "view",
        [](const Buffer& self)
        {
            BufferView bv;
            try
            {
                bv = self.view();
            }
            catch(const std::exception& e)
            {
                logger::error(PYUIPC_MSG("Error in get_buffer_view_func: {}", e.what()));
            }
            return bv;
        },
        R"(Get a view of the buffer.
Returns:
    BufferView: View of the buffer.)");
}
}  // namespace pyuipc::backend
