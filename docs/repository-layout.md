# Repository and package layout

The repository itself is a colcon workspace. ROS packages live under `src/`, the
upstream model checkout lives outside `src/`, and container files live at the
repository root.

## Layout

```text
trackvla_ros2/
├── README.md
├── compose.yaml
├── .env.example                    # copy to .env, which is never committed
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── requirements/
│   ├── torch-cu128.txt             # pinned torch, cu128 index
│   ├── inference.txt               # build input
│   └── inference-lock.txt          # resolved versions, informational
├── docs/
│   ├── architecture.md
│   ├── repository-layout.md
│   ├── environment.md
│   ├── implementation-plan.md
│   ├── interfaces.md               # topics, action, parameters
│   ├── running.md                  # build, run, record
│   ├── provenance.md               # pinned upstream and model versions
│   └── decisions.md
├── scripts/
│   ├── build_workspace.sh
│   ├── check_gpu.sh
│   └── phase0_check.py
├── src/
│   ├── vla_tracking_interfaces/    # ament_cmake: msg and action only
│   ├── vla_tracking/               # ament_python: nodes, contract, launch
│   └── vla_tracking_omtrackvla/    # ament_python: the OmTrackVLA adapter
├── third_party/
│   ├── COLCON_IGNORE
│   └── OmTrackVLA/                 # pinned Git submodule
├── cache/                          # ignored runtime caches
├── build/                          # ignored colcon output
├── install/                        # ignored colcon output
└── log/                            # ignored colcon output
```

Tests live inside each package's `test/` directory rather than in a top-level
`test/integration/`, because that is where `colcon test` finds and runs them.
The fixtures they share are installed as `vla_tracking.testing`, so an adapter
package can drive the same pipeline against real weights without copying the
scaffolding.

## ROS packages

### `vla_tracking_interfaces`

An `ament_cmake` interface-only package. It contains no executable node and does
not need its own container.

### `vla_tracking`

An `ament_python` package containing the generic ROS nodes, backend contract,
trajectory validation/conversion, launch file, and system configuration. Launch
and config remain here rather than creating a separate `bringup` or `config`
package while the system is small.

Expected console scripts:

```text
vla_inference_node
trajectory_executor_node
```

### `vla_tracking_omtrackvla`

An `ament_python` adapter package containing all knowledge of OmTrackVLA's
loader, preprocessing, temporal-token state, checkpoint configuration, and output
conversion. It depends on `vla_tracking`'s backend contract but generic packages
do not depend on it.

## Upstream source and model artifacts

The official OmTrackVLA repository is added as a pinned Git submodule under
`third_party/OmTrackVLA`. A `COLCON_IGNORE` marker prevents colcon from treating
its Habitat tree as workspace packages.

The adapter receives the upstream path from configuration; it must not hard-code
a developer home directory. Project-owned changes stay in the adapter. If an
upstream modification becomes unavoidable, it should be maintained in an
explicit fork rather than as uncommitted submodule edits.

Checkpoints and caches are mounted at runtime and are never committed:

```text
cache/huggingface/
cache/torch/
```

## Generated and ignored paths

The following are not versioned:

```text
build/
install/
log/
cache/
.env
__pycache__/
```
