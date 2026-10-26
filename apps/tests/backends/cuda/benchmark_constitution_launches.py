"""Prepare an isolated baseline, or compare production CUDA launch libraries."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[4]
BASE_REVISION = 'ffd572193cfbe2eb02e7c44a3f9890e14cd619d7'
LAUNCHES = (
    ('affine_body/constitutions/affine_body_fixed_joint.cu', 'affine_body_fixed_joint', None),
    ('finite_element/constitutions/arap_3d.cu', 'arap_3d', None),
    ('finite_element/constitutions/hookean_spring_1d.cu', 'hookean_spring_1d', None),
    ('finite_element/constitutions/kirchhoff_rod_bending.cu', 'kirchhoff_rod_bending', None),
    ('finite_element/constitutions/strain_plastic_discrete_shell_bending.cu', 'strain_plastic_bending', None),
    ('inter_primitive_effect_system/constitutions/soft_vertex_stitch.cu', 'soft_vertex_stitch', '2, 3'),
    ('inter_primitive_effect_system/constitutions/soft_vertex_edge_stitch.cu', 'soft_vertex_edge_stitch', '3, 6'),
    ('inter_primitive_effect_system/constitutions/soft_vertex_triangle_stitch.cu', 'soft_vertex_triangle_stitch', '4, 10'),
)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_baseline(directory: Path, reference: str) -> None:
    directory = directory.resolve()
    if directory == ROOT:
        raise SystemExit('Use a separate clean checkout for the baseline.')
    def git(*args: str) -> str:
        return subprocess.check_output(['git', '-C', str(directory), *args], text=True).strip()
    if Path(git('rev-parse', '--show-toplevel')).resolve() != directory:
        raise SystemExit('Baseline directory must be the root of a Git checkout.')
    if git('status', '--porcelain'):
        raise SystemExit('Baseline checkout has local changes; leave them intact.')
    if git('rev-parse', 'HEAD') != git('rev-parse', reference):
        raise SystemExit('Baseline HEAD does not match --reference.')
    header = ROOT / 'src/backends/cuda/utils/remaining_material_launch.h'
    sources = {}
    for relative, api, template in LAUNCHES:
        path = directory / 'src/backends/cuda' / relative
        text = path.read_text()
        match = re.search(r'__global__ void (\w*gradient_hessian_kernel)\((.*?)\)\s*\{', text, re.S)
        if not match:
            raise SystemExit(f'Cannot locate original kernel in {path}')
        parameters = match.group(2)
        pieces, begin, depth = [], 0, 0
        for index, char in enumerate(parameters):
            depth += (char == '<') - (char == '>')
            if char == ',' and depth == 0:
                pieces.append(parameters[begin:index].strip())
                begin = index + 1
        pieces.append(parameters[begin:].strip())
        fields = [re.search(r'(\w+)$', piece).group(1) for piece in pieces]
        if not fields or fields[-1] != 'n':
            raise SystemExit(f'Unsupported kernel arguments in {path}')
        declaration = re.search(
            r'void launch_' + api + r'_gradient_hessian\(\s*const (\w+)& info\);', header.read_text()
        )
        if not declaration:
            raise SystemExit(f'Cannot find launch declaration for {api}')
        count = next(field for field in fields if field in ('body_ids', 'indices', 'hinges', 'stencils', 'topos'))
        kernel = match.group(1) + ('<' + template + '>' if template else '')
        arguments = ',\n'.join('        info.' + name for name in fields[:-1]) + ',\n        n);'
        wrapper = (
            f'\n\nvoid launch_{api}_gradient_hessian(const {declaration.group(1)}& info)\n{{\n'
            f'    int n = static_cast<int>(info.{count}.size());\n    if(n <= 0) return;\n'
            f'    auto k = {kernel};\n'
            '    int bs = info.block_size > 0 ? info.block_size : cuda_tool::best_block_dim(k);\n'
            '    k<<<(n + bs - 1) / bs, bs, 0, nullptr>>>(\n' + arguments + '\n}\n'
        )
        position = text.index('}  // namespace', match.end()) + len('}  // namespace')
        sources[path] = '#include <utils/remaining_material_launch.h>\n' + text[:position] + wrapper + text[position:]
    # All checks above finish before any source is changed. Device bodies and
    # existing constitution dispatch remain exactly as in the reference.
    target = directory / header.relative_to(ROOT)
    target.write_text(header.read_text())
    for path, text in sources.items():
        path.write_text(text)
    print(f'Prepared {len(sources)} baseline launch shims at {directory}.')
    print('Configure this checkout with matching release/CUDA options, then build: xmake build -j4 cuda')


def compare(args: argparse.Namespace) -> None:
    binary = args.binary.resolve()
    baseline = args.baseline.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    package_libraries = {
        str(path.parent) for path in (Path.home() / '.xmake/packages').rglob('*.so*')
        if path.name.startswith(('libspdlog.so', 'libtbb'))
    }
    cases = []
    for iteration in range(args.runs):
        for side in ('baseline', 'optimized'):
            trial = output / f'{iteration + 1}-{side}'
            trial.mkdir()
            library = baseline if side == 'baseline' else binary.parent / 'libuipc_backend_cuda.so'
            environment = os.environ.copy()
            for name in ('LD_PRELOAD', 'UIPC_ENGINE_DEFAULT_CONFIG', 'UIPC_REMAINING_TUNE_MODE',
                         'UIPC_REMAINING_TUNE_CTA', 'UIPC_REMAINING_BLOCKS'):
                environment.pop(name, None)
            environment['LD_LIBRARY_PATH'] = ':'.join([
                str(library.parent), str(binary.parent), *args.library_path, *sorted(package_libraries),
                environment.get('LD_LIBRARY_PATH', '')])
            environment['UIPC_REMAINING_BENCH_OUTPUT'] = str(trial / 'benchmark.json')
            if side == 'baseline' and args.baseline_blocks:
                environment['UIPC_REMAINING_BLOCKS'] = str(args.baseline_blocks.resolve())
            command = [str(binary), 'remaining production material CUDA-event benchmark',
                       '--reporter', 'junit', '--out', str(trial / 'results.xml'), '--log-level', 'warn']
            observations, mapped = [], set()
            stop = threading.Event()
            with (trial / 'run.log').open('w') as log:
                child = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
                def observe() -> None:
                    with (trial / 'gpu.jsonl').open('w', buffering=1) as gpu_log:
                        while not stop.is_set():
                            row = {'unix_ns': time.time_ns(), 'load': os.getloadavg()}
                            try:
                                for key, query in [('gpu', '--query-gpu=utilization.gpu,memory.used,clocks.sm'),
                                                   ('processes', '--query-compute-apps=pid,process_name,used_memory')]:
                                    result = subprocess.run(['nvidia-smi', query, '--format=csv,noheader,nounits'],
                                                            text=True, capture_output=True, timeout=5)
                                    row[key] = result.stdout.strip()
                                maps = Path(f'/proc/{child.pid}/maps').read_text()
                                mapped.update(line.split()[-1] for line in maps.splitlines()
                                              if 'libuipc_backend_cuda.so' in line)
                            except (OSError, subprocess.TimeoutExpired) as error:
                                row['observation_error'] = str(error)
                            observations.append(row)
                            gpu_log.write(json.dumps(row) + '\n')
                            stop.wait(0.5)
                observer = threading.Thread(target=observe, daemon=True)
                observer.start()
                returncode = child.wait()
                stop.set()
                observer.join(timeout=12)
            identity = {'variant': side, 'iteration': iteration + 1, 'returncode': returncode,
                        'binary_sha256': sha(binary), 'library_sha256': sha(library),
                        'mapped_cuda_libraries': sorted(mapped), 'gpu_samples': len(observations)}
            (trial / 'identity.json').write_text(json.dumps(identity, indent=2) + '\n')
            if returncode or mapped != {str(library)}:
                raise SystemExit(f'Execution or library identity failed: {trial}')
            rows = json.loads((trial / 'benchmark.json').read_text())
            cases.append((side, rows))
            print(f'{iteration + 1}/{args.runs} {side} complete', flush=True)
    summary = []
    for index in range(len(cases[0][1])):
        group = {side: [rows[index]['median_ms'] for name, rows in cases if name == side]
                 for side in ('baseline', 'optimized')}
        summary.append({'material': cases[0][1][index]['material'],
                        'gradient_only': cases[0][1][index]['gradient_only'],
                        'medians_ms': {name: statistics.median(values) for name, values in group.items()},
                        'ranges_ms': {name: [min(values), max(values)] for name, values in group.items()},
                        'speedup': statistics.median(group['baseline']) / statistics.median(group['optimized'])})
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    prepare = modes.add_parser('prepare-baseline')
    prepare.add_argument('directory', type=Path)
    prepare.add_argument('--reference', default=BASE_REVISION)
    run = modes.add_parser('compare')
    run.add_argument('--baseline', required=True, type=Path)
    run.add_argument('--binary', type=Path, default=ROOT / 'build/xmake-cuda134/linux/x86_64/release/uipc_test_backend_cuda')
    run.add_argument('--baseline-blocks', type=Path)
    run.add_argument('--library-path', action='append', default=[])
    run.add_argument('--runs', type=int, default=5)
    run.add_argument('--output', required=True, type=Path)
    options = parser.parse_args()
    if options.mode == 'prepare-baseline':
        prepare_baseline(options.directory, options.reference)
    else:
        if options.runs < 1:
            parser.error('--runs must be positive')
        compare(options)
