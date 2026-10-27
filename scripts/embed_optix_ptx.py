"""Embed the OptiX PTX module as an exact, NUL-terminated C++ string."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    ptx = args.input.read_text(encoding="utf-8")
    delimiter = "uipc_optix_ptx"
    if f'){delimiter}"' in ptx:
        raise ValueError("PTX contains the reserved raw-string delimiter")
    header = (
        "// Generated from the OptiX device program. Do not edit.\n"
        "#pragma once\n"
        "#include <cstddef>\n\n"
        f'inline constexpr char uipc_optix_ef_device_ptx[] = R"{delimiter}('
        f'{ptx}){delimiter}";\n'
        "inline constexpr std::size_t uipc_optix_ef_device_ptx_size =\n"
        "    sizeof(uipc_optix_ef_device_ptx) - 1;\n"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.is_file() or args.output.read_text(encoding="utf-8") != header:
        args.output.write_text(header, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
