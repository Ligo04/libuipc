#include <app/app.h>
#include <cuda_tool/stream.h>
#include <utils/fixed_bank_soa_evd.h>
#include <utils/make_spd.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
template <typename T>
struct Allocation
{
    T* data = nullptr;
    explicit Allocation(std::size_t count)
    {
        CUDA_TOOL_CHECK(cudaMalloc(&data, count * sizeof(T)));
    }
    ~Allocation() { cudaFree(data); }
    Allocation(const Allocation&) = delete;
    Allocation& operator=(const Allocation&) = delete;
};

template <int N>
__global__ void legacy_spd_kernel(const Float* input, Float* output, int* status, int count)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= count)
        return;
    const std::size_t offset = static_cast<std::size_t>(i) * N * N;
    Matrix<Float, N, N> H;
    for(int col = 0; col < N; ++col)
        for(int row = 0; row < N; ++row)
            H(row, col) = input[offset + col * N + row];
    make_spd(H);
    for(int col = 0; col < N; ++col)
        for(int row = 0; row < N; ++row)
            output[offset + col * N + row] = H(row, col);
    status[i] = 0;
}

template <int N, int LanePitch>
__global__ void shared_spd_kernel(const Float* input, Float* output, int* status, int count)
{
    extern __shared__ Float storage[];
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= count)
        return;
    const std::size_t offset = static_cast<std::size_t>(i) * N * N;
    FixedBankSoAMap<N, LanePitch> workspace(storage + threadIdx.x);
    for(int col = 0; col < N; ++col)
        for(int row = 0; row < N; ++row)
            workspace(row, col) = input[offset + col * N + row];
    Matrix<Float, N, N> projected;
    const bool success = make_spd_fixed_bank_shared_upper_fma<N>(workspace, projected);
    for(int col = 0; col < N; ++col)
        for(int row = 0; row < N; ++row)
            output[offset + col * N + row] = projected(row, col);
    status[i] = success ? 0 : 1;
}

struct Configuration
{
    std::string implementation;
    int block_size;
    std::size_t shared_bytes;
    cudaFuncAttributes attributes{};
    int active_blocks_per_sm = 0;
    std::function<void(const Float*, Float*, int*, int)> launch;
    Json description() const
    {
        return {{"implementation", implementation}, {"block_size", block_size},
                {"dynamic_shared_bytes", shared_bytes},
                {"registers_per_thread", attributes.numRegs},
                {"local_bytes_per_thread", attributes.localSizeBytes},
                {"static_shared_bytes", attributes.sharedSizeBytes},
                {"active_blocks_per_sm", active_blocks_per_sm}};
    }
};

template <int N, int BlockSize>
void add_shared_configuration(std::vector<Configuration>& configs,
                              const cudaDeviceProp& props)
{
    constexpr std::size_t bytes = N * N * BlockSize * sizeof(Float);
    const auto max_shared = std::max(props.sharedMemPerBlock, props.sharedMemPerBlockOptin);
    if(bytes > max_shared)
        return;
    Configuration c{"fixed_bank_shared", BlockSize, bytes};
    CUDA_TOOL_CHECK(cudaFuncGetAttributes(&c.attributes, shared_spd_kernel<N, BlockSize>));
    if(BlockSize > c.attributes.maxThreadsPerBlock)
        return;
    if(bytes > props.sharedMemPerBlock)
        CUDA_TOOL_CHECK(cudaFuncSetAttribute(shared_spd_kernel<N, BlockSize>,
                                             cudaFuncAttributeMaxDynamicSharedMemorySize,
                                             static_cast<int>(bytes)));
    CUDA_TOOL_CHECK(cudaOccupancyMaxActiveBlocksPerMultiprocessor(
        &c.active_blocks_per_sm, shared_spd_kernel<N, BlockSize>, BlockSize, bytes));
    if(!c.active_blocks_per_sm)
        return;
    c.launch = [](const Float* input, Float* output, int* status, int count)
    {
        shared_spd_kernel<N, BlockSize><<<(count + BlockSize - 1) / BlockSize, BlockSize, bytes>>>(
            input, output, status, count);
    };
    configs.push_back(std::move(c));
}

template <int N>
std::vector<Configuration> configurations(const cudaDeviceProp& props)
{
    std::vector<Configuration> result;
    for(int block : {8, 16, 32, 64, 128, 256, 512})
    {
        Configuration c{"legacy", block, 0};
        CUDA_TOOL_CHECK(cudaFuncGetAttributes(&c.attributes, legacy_spd_kernel<N>));
        if(block > c.attributes.maxThreadsPerBlock)
            continue;
        CUDA_TOOL_CHECK(cudaOccupancyMaxActiveBlocksPerMultiprocessor(
            &c.active_blocks_per_sm, legacy_spd_kernel<N>, block, 0));
        if(!c.active_blocks_per_sm)
            continue;
        c.launch = [block](const Float* input, Float* output, int* status, int count)
        {
            legacy_spd_kernel<N><<<(count + block - 1) / block, block>>>(input, output, status, count);
        };
        result.push_back(std::move(c));
    }
    add_shared_configuration<N, 8>(result, props);
    add_shared_configuration<N, 16>(result, props);
    add_shared_configuration<N, 32>(result, props);
    add_shared_configuration<N, 64>(result, props);
    add_shared_configuration<N, 128>(result, props);
    add_shared_configuration<N, 256>(result, props);
    add_shared_configuration<N, 512>(result, props);
    return result;
}

template <int N>
std::vector<Float> inputs(int count, bool edge_cases)
{
    std::mt19937_64 generator(0x5eed5bdULL + N);
    std::uniform_real_distribution<Float> uniform(-1, 1);
    std::vector<Float> result(static_cast<std::size_t>(count) * N * N);
    for(int i = 0; i < count; ++i)
    {
        Matrix<Float, N, N> H;
        for(int col = 0; col < N; ++col)
            for(int row = col; row < N; ++row)
                H(row, col) = H(col, row) = uniform(generator);
        if(edge_cases)
        {
            if(i == 0)
                H.setZero();
            else if(i == 1)
                H = -Matrix<Float, N, N>::Identity();
            else if(i == 2)
                H.setIdentity();
            else if(i == 3)
            {
                H.setZero();
                for(int k = 0; k < N; ++k)
                    H(k, k) = k < N / 2 ? -2 : 1;
            }
            else if(i == 4)
            {
                Vector<Float, N> v;
                for(int k = 0; k < N; ++k)
                    v(k) = k + 1;
                H = v * v.transpose();
            }
            else if(i == 5)
                H *= 1e-12;
            else if(i == 6)
                H *= 1e6;
        }
        Eigen::Map<Matrix<Float, N, N>>(result.data() + static_cast<std::size_t>(i) * N * N) = H;
    }
    return result;
}

template <int N>
Json check_against_cpu(const cudaDeviceProp& props)
{
    constexpr int count = 257;
    using MatrixN = Matrix<Float, N, N>;
    auto input = inputs<N>(count, true);
    std::vector<MatrixN> expected(count);
    for(int i = 0; i < count; ++i)
    {
        Eigen::Map<const MatrixN> H(input.data() + static_cast<std::size_t>(i) * N * N);
        Eigen::SelfAdjointEigenSolver<MatrixN> solver(H);
        REQUIRE(solver.info() == Eigen::Success);
        expected[i] = solver.eigenvectors() * solver.eigenvalues().cwiseMax(Float(0)).asDiagonal()
                      * solver.eigenvectors().transpose();
    }
    Allocation<Float> device_input(input.size()), device_output(input.size());
    Allocation<int> device_status(count);
    CUDA_TOOL_CHECK(cudaMemcpy(device_input.data, input.data(), input.size() * sizeof(Float),
                               cudaMemcpyHostToDevice));
    Json results = Json::array();
    for(const auto& c : configurations<N>(props))
    {
        INFO("N=" << N << " implementation=" << c.implementation << " block=" << c.block_size);
        c.launch(device_input.data, device_output.data, device_status.data, count);
        CUDA_TOOL_CHECK(cudaGetLastError());
        CUDA_TOOL_CHECK(cudaDeviceSynchronize());
        std::vector<Float> output(input.size());
        std::vector<int> status(count);
        CUDA_TOOL_CHECK(cudaMemcpy(output.data(), device_output.data, output.size() * sizeof(Float),
                                   cudaMemcpyDeviceToHost));
        CUDA_TOOL_CHECK(cudaMemcpy(status.data(), device_status.data, count * sizeof(int),
                                   cudaMemcpyDeviceToHost));
        Float max_relative_error = 0;
        Float min_normalized_eigenvalue = 0;
        int worst_matrix_id = 0;
        // Eigen documents ~1e-8 worst-case double eigenvalue errors for its
        // 3x3 computeDirect path. Keep QR-based projections at the stricter
        // tolerance and record both the error and the applicable bound.
        const Float tolerance = N == 3 && c.implementation == "legacy" ? 1e-8 : 1e-12;
        for(int i = 0; i < count; ++i)
        {
            INFO("matrix_id=" << i);
            REQUIRE(status[i] == 0);
            Eigen::Map<const MatrixN> H(output.data() + static_cast<std::size_t>(i) * N * N);
            REQUIRE(H.allFinite());
            const Float scale = std::max(Float(1), expected[i].norm());
            const Float error = (H - expected[i]).norm() / scale;
            if(error > max_relative_error)
            {
                max_relative_error = error;
                worst_matrix_id = i;
            }
            REQUIRE(error <= tolerance);
            REQUIRE((H - H.transpose()).norm() / scale <= 1e-12);
            Eigen::SelfAdjointEigenSolver<MatrixN> solver(H);
            REQUIRE(solver.info() == Eigen::Success);
            const Float min_value = solver.eigenvalues().minCoeff() / scale;
            min_normalized_eigenvalue = std::min(min_normalized_eigenvalue, min_value);
            REQUIRE(min_value >= -1e-12);
        }
        auto row = c.description();
        row["max_relative_error"] = max_relative_error;
        row["relative_error_tolerance"] = tolerance;
        row["meets_1e_minus_12"] = max_relative_error <= 1e-12;
        row["worst_matrix_id"] = worst_matrix_id;
        row["min_normalized_eigenvalue"] = min_normalized_eigenvalue;
        results.push_back(row);
    }
    return {{"dimension", N}, {"matrix_count", count}, {"configurations", results}};
}

double median(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) * 0.5;
}

double measure(const Configuration& c, const Float* input, Float* output,
               int* status, int count, int iterations)
{
    cudaEvent_t start, end;
    CUDA_TOOL_CHECK(cudaEventCreate(&start));
    CUDA_TOOL_CHECK(cudaEventCreate(&end));
    CUDA_TOOL_CHECK(cudaEventRecord(start));
    for(int i = 0; i < iterations; ++i)
        c.launch(input, output, status, count);
    CUDA_TOOL_CHECK(cudaEventRecord(end));
    CUDA_TOOL_CHECK(cudaEventSynchronize(end));
    CUDA_TOOL_CHECK(cudaGetLastError());
    float elapsed = 0;
    CUDA_TOOL_CHECK(cudaEventElapsedTime(&elapsed, start, end));
    CUDA_TOOL_CHECK(cudaEventDestroy(start));
    CUDA_TOOL_CHECK(cudaEventDestroy(end));
    return elapsed / iterations;
}

template <int N>
Json benchmark(const cudaDeviceProp& props, bool profile_only)
{
    constexpr int count = 65536;
    auto input = inputs<N>(count, false);
    Allocation<Float> device_input(input.size()), device_output(input.size());
    Allocation<int> device_status(count);
    CUDA_TOOL_CHECK(cudaMemcpy(device_input.data, input.data(), input.size() * sizeof(Float),
                               cudaMemcpyHostToDevice));
    auto configs = configurations<N>(props);
    std::vector<std::vector<double>> tuning(configs.size());
    for(const auto& c : configs)
        for(int i = 0; i < 3; ++i)
            c.launch(device_input.data, device_output.data, device_status.data, count);
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
    // Reverse the complete configuration order on alternating tuning rounds.
    for(int round = 0; round < 3; ++round)
        for(std::size_t step = 0; step < configs.size(); ++step)
        {
            const auto k = round % 2 ? configs.size() - 1 - step : step;
            tuning[k].push_back(measure(configs[k], device_input.data, device_output.data,
                                       device_status.data, count, 1));
        }
    std::size_t best_legacy = configs.size(), best_shared = configs.size();
    std::size_t legacy32 = configs.size(), shared32 = configs.size();
    Json all_tuning = Json::array();
    for(std::size_t k = 0; k < configs.size(); ++k)
    {
        auto row = configs[k].description();
        row["samples_ms"] = tuning[k];
        all_tuning.push_back(row);
        auto& best = configs[k].implementation == "legacy" ? best_legacy : best_shared;
        if(best == configs.size() || median(tuning[k]) < median(tuning[best]))
            best = k;
        if(configs[k].block_size == 32)
            (configs[k].implementation == "legacy" ? legacy32 : shared32) = k;
    }
    REQUIRE(best_legacy < configs.size());
    REQUIRE(best_shared < configs.size());
    REQUIRE(legacy32 < configs.size());
    REQUIRE(shared32 < configs.size());
    Json comparisons = Json::array();
    for(int comparison = 0; comparison < 2; ++comparison)
    {
        const auto left = comparison == 0 ? best_legacy : legacy32;
        const auto right = comparison == 0 ? best_shared : shared32;
        const std::array<std::size_t, 2> selected{left, right};
        const int iterations = std::clamp(static_cast<int>(std::ceil(
            2.0 / std::min(median(tuning[left]), median(tuning[right])))), 1, 32);
        std::array<std::vector<double>, 2> samples;
        for(int round = 0; round < (profile_only ? 1 : 7); ++round)
        {
            for(int step = 0; step < 2; ++step)
            {
                const int side = round % 2 ? 1 - step : step;
                samples[side].push_back(measure(configs[selected[side]], device_input.data,
                                                device_output.data, device_status.data,
                                                count, iterations));
            }
        }
        Json sides = Json::array();
        std::vector<Float> legacy_output;
        for(int side = 0; side < 2; ++side)
        {
            const auto& c = configs[selected[side]];
            c.launch(device_input.data, device_output.data, device_status.data, count);
            CUDA_TOOL_CHECK(cudaGetLastError());
            CUDA_TOOL_CHECK(cudaDeviceSynchronize());
            std::vector<Float> output(input.size());
            std::vector<int> status(count);
            CUDA_TOOL_CHECK(cudaMemcpy(output.data(), device_output.data, output.size() * sizeof(Float),
                                       cudaMemcpyDeviceToHost));
            CUDA_TOOL_CHECK(cudaMemcpy(status.data(), device_status.data, count * sizeof(int),
                                       cudaMemcpyDeviceToHost));
            REQUIRE(std::all_of(status.begin(), status.end(), [](int v) { return v == 0; }));
            REQUIRE(std::all_of(output.begin(), output.end(), [](Float v) { return std::isfinite(v); }));
            Float max_difference = 0;
            if(side == 0)
                legacy_output = output;
            else
                for(int i = 0; i < count; ++i)
                {
                    Eigen::Map<const Matrix<Float, N, N>> a(legacy_output.data() + static_cast<std::size_t>(i) * N * N);
                    Eigen::Map<const Matrix<Float, N, N>> b(output.data() + static_cast<std::size_t>(i) * N * N);
                    max_difference = std::max(max_difference, (a - b).norm() / std::max(Float(1), a.norm()));
                }
            REQUIRE(max_difference <= (N == 3 ? 1e-8 : 1e-12));
            auto row = c.description();
            row["samples_ms"] = samples[side];
            row["median_ms"] = median(samples[side]);
            row["min_ms"] = *std::min_element(samples[side].begin(), samples[side].end());
            row["max_ms"] = *std::max_element(samples[side].begin(), samples[side].end());
            row["nanoseconds_per_matrix"] = median(samples[side]) * 1e6 / count;
            row["max_relative_difference_from_legacy"] = max_difference;
            row["finite_outputs"] = output.size();
            sides.push_back(row);
        }
        comparisons.push_back({{"mode", comparison == 0 ? "independently_tuned" : "same_block_32"},
                               {"iterations_per_sample", iterations}, {"sides", sides},
                               {"speedup", median(samples[0]) / median(samples[1])}});
    }
    Json result{{"dimension", N}, {"matrix_count", count}, {"tuning", all_tuning},
                {"comparisons", comparisons}};
    std::cout << "SPD_BENCH_RESULT " << result.dump() << std::endl;
    return result;
}

cudaDeviceProp setup_device()
{
    CUDA_TOOL_CHECK(cudaDeviceSetLimit(cudaLimitStackSize, 32 * 1024));
    int device = 0;
    CUDA_TOOL_CHECK(cudaGetDevice(&device));
    cudaDeviceProp props{};
    CUDA_TOOL_CHECK(cudaGetDeviceProperties(&props, device));
    return props;
}
}  // namespace

TEST_CASE("make_spd projections match CPU Eigen", "[cuda][make_spd]")
{
    const auto props = setup_device();
    Json results = Json::array();
    results.push_back(check_against_cpu<3>(props));
    results.push_back(check_against_cpu<6>(props));
    results.push_back(check_against_cpu<9>(props));
    results.push_back(check_against_cpu<12>(props));
    if(const char* filename = std::getenv("UIPC_SPD_CORRECTNESS_OUTPUT"))
        std::ofstream(filename) << results.dump(2) << '\n';
}

TEST_CASE("make_spd CUDA-event benchmark", "[.benchmark][cuda][make_spd_benchmark]")
{
    const auto props = setup_device();
    const bool profile_only = std::getenv("UIPC_SPD_PROFILE_ONLY") != nullptr;
    Json results{{"gpu", props.name}, {"multiprocessors", props.multiProcessorCount},
                 {"compute_capability", {props.major, props.minor}},
                 {"precision", "float64"}, {"input_layout", "column-major AoS"},
                 {"measurement", "CUDA events: input loads, complete PSD projection, output stores; excludes transfers/allocations"},
                 {"profile_only", profile_only}, {"dimensions", Json::array()}};
    results["dimensions"].push_back(benchmark<3>(props, profile_only));
    results["dimensions"].push_back(benchmark<6>(props, profile_only));
    results["dimensions"].push_back(benchmark<9>(props, profile_only));
    results["dimensions"].push_back(benchmark<12>(props, profile_only));
    if(const char* filename = std::getenv("UIPC_SPD_BENCH_OUTPUT"))
        std::ofstream(filename) << results.dump(2) << '\n';
}
