# Autonomy Traffic Lab

Autonomy Traffic Lab is a portfolio-grade autonomous-driving simulation project. It combines a deterministic Python traffic-safety core with an optional [CARLA](https://carla.org/) 3D backend.

The project is designed to answer concrete safety questions in repeatable scenarios:

- When should a vehicle stop for a red light, a crossing pedestrian, or a blocked conflict zone?
- When is it safe for pedestrians to enter a crosswalk?
- How do traffic policies behave as vehicle and pedestrian populations scale?
- Can every safety decision be explained, tested, logged, and replayed?

## Project status

The repository starts with the Python package and development contract. The implementation will be built in small, reviewable milestones:

1. deterministic domain model and simulation clock;
2. vehicle, pedestrian, crosswalk, and traffic-light state machines;
3. safety policy engine with explicit invariants;
4. scenario configuration and scalable actor spawning;
5. CARLA adapter for the 3D world and sensors;
6. metrics, event logs, replay, visual diagnostics, and CI.

## Recommended software

| Component | Purpose | Required now? |
| --- | --- | --- |
| Python 3.11 or 3.12 | Core simulator and tooling | Yes |
| Git + GitHub | Version control and portfolio history | Yes |
| VS Code or PyCharm | Development and debugging | Yes |
| CARLA 3D simulator | Roads, vehicles, pedestrians, traffic lights, and sensors | Later integration |
| NVIDIA GPU + current driver | Practical real-time CARLA rendering | Recommended for 3D runs |
| Docker + NVIDIA Container Toolkit | Reproducible CARLA runtime on Linux | Optional |
| Unreal Engine | Custom CARLA maps/assets only | Optional |
| Blender | Custom 3D assets only | Optional |
| ROS 2 | Robotics middleware integration | Future extension |

CARLA is intentionally optional during core development. The safety engine must be testable in CI and on machines that do not have a graphics card or a CARLA installation.

## Quick start

```bash
python -m venv .venv

# macOS/Linux
source .venv/bin/activate

# Windows PowerShell
# .venv\\Scripts\\Activate.ps1

python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
pytest
autodrive-sim doctor
autodrive-sim run --scenario scenarios/urban_crosswalk.toml --steps 100
```

The `run` command prints a JSON metrics summary. Add `--events artifacts/run.jsonl` to persist all policy decisions and signal changes.

For a watchable CARLA run, start CARLA first and use the native Traffic
Manager for map-aware lane following, traffic lights, junction priority, and
vehicle/walker collision avoidance:

```powershell
.\.carla-venv\Scripts\python.exe -m autodrive_sim run `
  --backend carla `
  --scenario .\scenarios\carla_live_crossing.toml `
  --live `
  --compact-traffic `
  --traffic-manager `
  --duration-s 120
```

Use `--duration-s 0` to run until `Ctrl+C`. The compact layout is a visual
demo option; the normal CARLA spawn layout remains available for experiments.

The CARLA client is installed separately because its Python wheel must match the CARLA server package and the target operating system. We will pin and document the compatible version once the target machine is known.

## Engineering goals

- deterministic seeds for reproducible experiments;
- explicit safety invariants instead of hidden behavior;
- a backend interface that keeps business logic independent from CARLA;
- typed configuration and structured event logs;
- unit, integration, and scenario-level tests;
- CI that validates the core without requiring a GPU;
- clear experiment reports suitable for a software-engineering portfolio.

## Repository layout

```text
src/autodrive_sim/   Python package
tests/               Fast automated tests
docs/                Architecture and milestone documentation
```

## Commit history

The repository will use meaningful milestone commits: each commit should represent a coherent capability, include tests where applicable, and be safe to review. We will not rewrite timestamps or fabricate activity; the quality of the engineering trail is more valuable than an artificial calendar.

