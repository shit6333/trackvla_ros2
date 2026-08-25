#!/usr/bin/env bash
# Fails loudly when CUDA is unavailable. The project has no CPU fallback.
set -euo pipefail

python3 - <<'PY'
import sys
import torch

print(f"python       {sys.version.split()[0]}")
print(f"torch        {torch.__version__}")
print(f"torch.cuda   {torch.version.cuda}")
print(f"cudnn        {torch.backends.cudnn.version()}")

if not torch.cuda.is_available():
    raise SystemExit("FAIL: torch.cuda.is_available() is False")

name = torch.cuda.get_device_name(0)
arch_list = torch.cuda.get_arch_list()
capability = torch.cuda.get_device_capability(0)
required = f"sm_{capability[0]}{capability[1]}"

print(f"device       {name}")
print(f"capability   {required}")
print(f"arch list    {' '.join(arch_list)}")

if required not in arch_list:
    raise SystemExit(
        f"FAIL: this torch build has no {required} kernels for {name}"
    )

print("PASS: CUDA is available and the device architecture is supported")
PY
