/***************************************************************************
* Copyright (c) 2019, Martin Renou                                         *
*                                                                          *
* Distributed under the terms of the BSD 3-Clause License.                 *
*                                                                          *
* The full license is in the file LICENSE, distributed with this software. *
****************************************************************************/

#ifndef NANOBIND_JSON_HPP
#define NANOBIND_JSON_HPP

#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <nanobind/nanobind.h>

namespace py = nanobind;
namespace nl = nlohmann;

namespace pyjson
{
inline py::object from_json(const nl::json& j)
{
    if(j.is_null())
        return py::none();
    if(j.is_boolean())
        return py::bool_(j.get<bool>());
    if(j.is_number_unsigned())
        return py::int_(j.get<nl::json::number_unsigned_t>());
    if(j.is_number_integer())
        return py::int_(j.get<nl::json::number_integer_t>());
    if(j.is_number_float())
        return py::float_(j.get<double>());
    if(j.is_string())
        return py::str(j.get_ref<const std::string&>().c_str());
    if(j.is_array())
    {
        py::list result;
        for(const auto& value : j)
            result.append(from_json(value));
        return result;
    }

    py::dict result;
    for(auto it = j.cbegin(); it != j.cend(); ++it)
        result[py::str(it.key().c_str())] = from_json(it.value());
    return result;
}

inline nl::json to_json(const py::handle& obj)
{
    if(!obj.is_valid() || obj.is_none())
        return nullptr;
    if(py::isinstance<py::bool_>(obj))
        return py::cast<bool>(obj);
    if(py::isinstance<py::int_>(obj))
    {
        try
        {
            auto value = py::cast<nl::json::number_integer_t>(obj);
            if(py::int_(value).equal(obj))
                return value;
        }
        catch(...)
        {
        }
        try
        {
            auto value = py::cast<nl::json::number_unsigned_t>(obj);
            if(py::int_(value).equal(obj))
                return value;
        }
        catch(...)
        {
        }
        throw std::runtime_error("to_json received an integer outside nlohmann::json's integer range: "
                                 + py::cast<std::string>(py::repr(obj)));
    }
    if(py::isinstance<py::float_>(obj))
        return py::cast<double>(obj);
    if(py::isinstance<py::bytes>(obj))
    {
        auto base64 = py::module_::import_("base64");
        return py::cast<std::string>(base64.attr("b64encode")(obj).attr("decode")("utf-8"));
    }
    if(py::isinstance<py::str>(obj))
        return py::cast<std::string>(obj);
    if(py::isinstance<py::tuple>(obj) || py::isinstance<py::list>(obj))
    {
        auto result = nl::json::array();
        for(py::handle value : obj)
            result.push_back(to_json(value));
        return result;
    }
    if(py::isinstance<py::dict>(obj))
    {
        auto result = nl::json::object();
        for(py::handle key : obj)
            result[py::cast<std::string>(py::str(key))] = to_json(obj[key]);
        return result;
    }
    throw std::runtime_error("to_json does not support this Python object: "
                             + py::cast<std::string>(py::repr(obj)));
}
}  // namespace pyjson

namespace nanobind::detail
{
template <>
struct type_caster<nl::json>
{
    NB_TYPE_CASTER(nl::json, const_name("json"));

    bool from_python(handle src, uint32_t, cleanup_list*) noexcept
    {
        try
        {
            value = pyjson::to_json(src);
            return true;
        }
        catch(...)
        {
            PyErr_Clear();
            return false;
        }
    }

    static handle from_cpp(const nl::json& src, rv_policy, cleanup_list*) noexcept
    {
        try
        {
            return pyjson::from_json(src).release();
        }
        catch(const std::exception& e)
        {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return {};
        }
        catch(...)
        {
            PyErr_SetString(PyExc_RuntimeError, "failed to convert JSON to a Python object");
            return {};
        }
    }
};
}  // namespace nanobind::detail

#endif
