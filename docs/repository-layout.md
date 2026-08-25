# Repository and package layout

The repository itself is a colcon workspace. ROS packages live under `src/`, the
upstream model checkout lives outside `src/`, and container files live at the
repository root.

## Planned layout

```text
trackvla_ros2/
├── README.md
├── LICENSE
├── compose.yaml
├── .env.example
├── .gitignore
├── .dockerignore
├── .gitmodules
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── requirements/
│   ├── inference.txt
│   └── inference-lock.txt
├── docs/
│   ├── architecture.md
│   ├── repository-layout.md
│   ├── environment.md
│   ├── implementation-plan.md
│   └── decisions.md
├── src/
│   ├── vla_tracking_interfaces/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── msg/
│   │   │   ├── VlaTrajectory.msg
│   │   │   └── VlaStatus.msg
│   │   └── action/
│   │       └── TrackTarget.action
│   ├── vla_tracking/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/vla_tracking
│   │   ├── vla_tracking/
│   │   │   ├── inference_node.py
│   │   │   ├── trajectory_executor_node.py
│   │   │   ├── backend_interface.py
│   │   │   ├── backend_loader.py
│   │   │   ├── image_buffer.py
│   │   │   ├── trajectory_conversion.py
│   │   │   └── trajectory_validation.py
│   │   ├── launch/tracking.launch.py
│   │   ├── config/
│   │   │   ├── tracking.yaml
│   │   │   ├── velocity_smoother.yaml
│   │   │   └── collision_monitor.yaml
│   │   └── test/
│   └── vla_tracking_omtrackvla/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/vla_tracking_omtrackvla
│       ├── vla_tracking_omtrackvla/
│       │   ├── omtrackvla_backend.py
│       │   ├── model_loader.py
│       │   └── preprocessing.py
│       └── test/
├── third_party/
│   ├── COLCON_IGNORE
│   └── OmTrackVLA/                 # pinned Git submodule
├── scripts/
│   ├── build_workspace.sh
│   ├── run_tracking.sh
│   └── check_gpu.sh
├── test/integration/
├── cache/                          # ignored runtime caches
├── build/                          # ignored colcon output
├── install/                        # ignored colcon output
└── log/                            # ignored colcon output
```

Only planning documents exist initially. The code directories and files above
are created incrementally by the implementation phases.

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
