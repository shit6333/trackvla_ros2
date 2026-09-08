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
major, minor = torch.cuda.get_device_capability(0)
required = f"sm_{major}{minor}"

print(f"device       {name}")
print(f"capability   {required}")
print(f"arch list    {' '.join(arch_list)}")

# A CUDA binary built for capability X.y runs on any device X.z with z >= y,
# so a card is covered when the build carries a kernel for its major version
# with a minor no higher than its own. The cu128 wheels have sm_86 but no
# sm_89, for example, and an RTX 4070 (8.9) runs the sm_86 binaries.
supported = sorted(
    int(arch.split("_")[1]) for arch in arch_list if arch.startswith("sm_")
)
covered = [sm for sm in supported if sm // 10 == major and sm <= major * 10 + minor]
if not covered:
    raise SystemExit(
        f"FAIL: this torch build has no binaries a {required} device can run "
        f"({name}); its architectures are {' '.join(arch_list)}"
    )
if required not in arch_list:
    print(
        f"NOTE: no native {required} kernels; this device will run the "
        f"sm_{max(covered)} binaries, which CUDA guarantees to be compatible"
    )

print("PASS: CUDA is available and the device architecture is supported")
PY
