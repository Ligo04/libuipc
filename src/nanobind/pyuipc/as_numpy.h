#pragma once
#include <pyuipc/exception.h>
#include <nanobind/ndarray.h>
#include <uipc/common/span.h>
#include <uipc/common/log.h>
#include <Eigen/Core>
#include <algorithm>
#include <initializer_list>
#include <type_traits>

namespace pyuipc
{
namespace py = nanobind;
using namespace uipc;

template <typename T>
using PyArray = py::ndarray<py::numpy, T, py::c_contig>;

template <typename T>
using PyArrayView = py::ndarray<py::numpy, T>;

template <typename T>
PyArray<T> make_numpy_array(std::initializer_list<size_t> shape)
{
    static_assert(!std::is_const_v<T>);

    size_t count = 1;
    for(size_t extent : shape)
        count *= extent;

    // Keep a non-null owner even for an empty array so ownership remains
    // explicit and uniform across Python versions.
    T*          data = new T[std::max<size_t>(count, 1)];
    py::capsule owner(data,
                      [](void* ptr) noexcept { delete[] static_cast<T*>(ptr); });
    return PyArray<T>(data, shape, owner);
}

template <typename T, typename ArrayScalar>
span<T> as_span(PyArray<ArrayScalar> arr)
    requires(std::is_arithmetic_v<std::remove_const_t<T>>
             && std::is_same_v<std::remove_const_t<T>, std::remove_const_t<ArrayScalar>>)
{
    static_assert(std::is_const_v<T> || !std::is_const_v<ArrayScalar>);
    PYUIPC_ASSERT(arr.ndim() == 1, "array must be 1D, yours={}", arr.ndim());
    return span<T>(arr.data(), arr.size());
}

template <typename T, int M, int N, int Options>
auto as_numpy(const span<Eigen::Matrix<T, M, N, Options>>& values, py::handle owner)
    requires(M > 0 && N > 0)
{
    using MatrixT = Eigen::Matrix<T, M, N, Options>;

    int64_t row_stride = MatrixT::OuterStrideAtCompileTime;
    int64_t col_stride = MatrixT::InnerStrideAtCompileTime;
    if constexpr(!(MatrixT::Flags & Eigen::RowMajorBit))
        std::swap(row_stride, col_stride);

    return PyArrayView<T>(reinterpret_cast<T*>(values.data()),
                          {values.size(), M, N},
                          owner,
                          {static_cast<int64_t>(sizeof(MatrixT) / sizeof(T)), row_stride, col_stride});
}

template <typename T, int M, int N, int Options>
auto as_numpy(const span<const Eigen::Matrix<T, M, N, Options>>& values, py::handle owner)
    requires(M > 0 && N > 0)
{
    using MatrixT = Eigen::Matrix<T, M, N, Options>;

    int64_t row_stride = MatrixT::OuterStrideAtCompileTime;
    int64_t col_stride = MatrixT::InnerStrideAtCompileTime;
    if constexpr(!(MatrixT::Flags & Eigen::RowMajorBit))
        std::swap(row_stride, col_stride);

    return PyArrayView<const T>(
        reinterpret_cast<const T*>(values.data()),
        {values.size(), M, N},
        owner,
        {static_cast<int64_t>(sizeof(MatrixT) / sizeof(T)), row_stride, col_stride});
}

template <typename T>
auto as_numpy(const span<T>& values, py::handle owner)
{
    return PyArrayView<T>(values.data(), {values.size()}, owner);
}

template <typename MatrixT, typename ArrayScalar>
span<MatrixT> as_span_of(PyArray<ArrayScalar> arr)
    requires(std::remove_const_t<MatrixT>::RowsAtCompileTime > 0
             && std::remove_const_t<MatrixT>::ColsAtCompileTime > 0
             && std::is_same_v<typename std::remove_const_t<MatrixT>::Scalar, std::remove_const_t<ArrayScalar>>)
{
    using RawMatrixT       = std::remove_const_t<MatrixT>;
    constexpr int  Rows    = RawMatrixT::RowsAtCompileTime;
    constexpr int  Cols    = RawMatrixT::ColsAtCompileTime;
    constexpr bool IsConst = std::is_const_v<MatrixT>;

    static_assert(IsConst || !std::is_const_v<ArrayScalar>);

    if(arr.ndim() == 2)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT(arr.shape(1) == Rows * Cols,
                          "Shape mismatch, ask for shape=(N,{}), yours=({},{})",
                          Rows * Cols,
                          arr.shape(0),
                          arr.shape(1));
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 3D"));
        }
    }
    else if(arr.ndim() == 3)
    {
        PYUIPC_ASSERT(arr.shape(1) == Rows && arr.shape(2) == Cols,
                      "Shape mismatch, ask for shape=(N,{},{}), yours=({},{},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1),
                      arr.shape(2));
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 2D or 3D, yours={}", arr.ndim()));
    }

    return span<MatrixT>(reinterpret_cast<MatrixT*>(arr.data()), arr.shape(0));
}

template <typename MatrixT, typename ArrayScalar>
bool is_span_of(PyArray<ArrayScalar> arr)
    requires(std::remove_const_t<MatrixT>::RowsAtCompileTime > 0
             && std::remove_const_t<MatrixT>::ColsAtCompileTime > 0
             && std::is_same_v<typename std::remove_const_t<MatrixT>::Scalar, std::remove_const_t<ArrayScalar>>)
{
    using RawMatrixT   = std::remove_const_t<MatrixT>;
    constexpr int Rows = RawMatrixT::RowsAtCompileTime;
    constexpr int Cols = RawMatrixT::ColsAtCompileTime;

    if(arr.ndim() == 2)
    {
        if constexpr(Rows == 1 || Cols == 1)
            return arr.shape(1) == Rows * Cols;
        return false;
    }
    if(arr.ndim() == 3)
        return arr.shape(1) == Rows && arr.shape(2) == Cols;
    return false;
}

template <typename T, int M, int N, int Options>
auto as_numpy(const Matrix<T, M, N, Options>& matrix)
    requires(M > 0 && N > 0)
{
    auto result = make_numpy_array<T>({M, N});
    for(int i = 0; i < M; ++i)
        for(int j = 0; j < N; ++j)
            result.data()[i * N + j] = matrix(i, j);
    return result;
}

template <typename T, int M, int N, int Options>
auto as_numpy(Matrix<T, M, N, Options>& matrix)
    requires(M > 0 && N > 0)
{
    return as_numpy(std::as_const(matrix));
}

template <typename MatrixT, typename ArrayScalar>
MatrixT to_matrix(PyArray<ArrayScalar> arr)
    requires(MatrixT::RowsAtCompileTime > 0 && MatrixT::ColsAtCompileTime > 0
             && std::is_same_v<typename MatrixT::Scalar, std::remove_const_t<ArrayScalar>>)
{
    constexpr int Rows = MatrixT::RowsAtCompileTime;
    constexpr int Cols = MatrixT::ColsAtCompileTime;

    MatrixT matrix;
    if(arr.ndim() == 1)
    {
        if(Rows == 1 || Cols == 1)
        {
            PYUIPC_ASSERT(arr.size() == Rows * Cols,
                          "Shape mismatch, ask for shape=(N,{}), yours={}",
                          Rows * Cols,
                          arr.size());
        }
        else
        {
            throw PyException(PYUIPC_MSG("array must be 2D, yours={}", arr.ndim()));
        }

        for(int i = 0; i < std::max(Rows, Cols); ++i)
            matrix(i) = arr.data()[i];
    }
    else if(arr.ndim() == 2)
    {
        PYUIPC_ASSERT(arr.shape(0) == Rows && arr.shape(1) == Cols,
                      "Shape mismatch, ask for shape=({},{}), yours=({},{})",
                      Rows,
                      Cols,
                      arr.shape(0),
                      arr.shape(1));

        for(int i = 0; i < Rows; ++i)
            for(int j = 0; j < Cols; ++j)
                matrix(i, j) = arr.data()[i * Cols + j];
    }
    else
    {
        throw PyException(PYUIPC_MSG("array must be 1D or 2D, yours={}", arr.ndim()));
    }

    return matrix;
}
}  // namespace pyuipc
