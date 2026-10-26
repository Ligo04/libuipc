# SPD projection tests and benchmark

The default `[make_spd]` test compares both complete projection functions with
CPU Eigen for FP64 3x3, 6x6, 9x9 and 12x12 matrices. It checks all supported
CTA configurations, finite/symmetric/PSD outputs, and partial-block tails.
The existing `[fixed_size_evd]` test covers the shared EVD separately.

```sh
xmake build -j4 backend_cuda
xmake run -w "$PWD" backend_cuda '[make_spd],[fixed_size_evd]' --log-level warn
UIPC_SPD_BENCH_OUTPUT="$PWD/build/verification/spd-result.json" xmake run -w "$PWD" backend_cuda 'make_spd CUDA-event benchmark' --log-level warn
```

The benchmark is hidden from the default suite. Each process uses identical
batches of 65,536 seeded symmetric matrices, column-major AoS buffers, three
warmup launches, CTA tuning over supported 8..512-thread configurations, and
seven alternating CUDA-event samples. It reports independently tuned and
matched-32-thread comparisons. Timing includes complete projection and output
stores; allocation, transfers and CPU validation are outside the timed region.
All output coefficients are consumed and validated after timing. Repeat the
whole process and retain GPU-load observations before drawing conclusions.

`UIPC_SPD_CORRECTNESS_OUTPUT` writes numeric errors to JSON.
`UIPC_SPD_PROFILE_ONLY=1` reduces the formal rounds for a separate profiler run;
those samples must not be included in normal timing results.

Eigen's legacy 3x3 computeDirect path has documented double eigenvalue errors
around 1e-8 in difficult cases. The test records its actual projected-matrix
error and uses a 1e-8 bound for that path, while QR-based paths retain 1e-12.
Symmetry and normalized minimum-eigenvalue checks retain 1e-12 for both.
Eigenvalue and projected-matrix error are distinct metrics.

On RTX 5090 / CUDA 13.4, five independent processes (seven samples each) gave:

| Size | Legacy ms/batch | Shared ms/batch | Legacy/shared |
| --- | ---: | ---: | ---: |
| 3x3 | 0.040980 | 0.119798 | 0.342x |
| 6x6 | 0.691080 | 0.664976 | 1.039x |
| 9x9 | 1.942112 | 1.749984 | 1.110x |
| 12x12 | 4.046592 | 4.966208 | 0.815x |

These synthetic, warm-buffer component measurements do not establish a universal
replacement rule: 3x3 and 12x12 were slower with shared storage, 6x6 was close to
variation, and 9x9 had a modest benefit. No other CUDA compute process was seen
in 27 formal-run samples, which does not exclude graphics activity. They do
not measure production Hessian-generation/assembly fusion or translation-free
reduction. Nsight Systems verified execution separately; hardware bottleneck
counters were not collected. The full default CUDA suite passed 73 cases and
27,311,453 assertions after adding this test. Raw trial data remains local in
build/verification/spd-benchmark-20261026-155405.
