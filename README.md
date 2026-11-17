# Autonomous Driving Safety Simulator

This project is a small traffic-safety simulator with an optional CARLA 3D backend. It focuses on everyday intersection problems: traffic lights, crosswalks, pedestrians, vehicle yielding, collisions, and near misses.

There are two ways to use it:

- **Core simulator:** a fast, deterministic mode for tests and repeatable experiments. It does not need CARLA.
- **CARLA mode:** a live 3D simulation with vehicles, pedestrians, traffic lights, and Traffic Manager.

## Requirements

For the core simulator:

- Python 3.11 or newer
- Git (optional, but useful for development)

For the 3D simulation:

- CARLA 0.9.16, or another supported CARLA version
- The matching CARLA Python package
- A computer that can run CARLA in real time

CARLA is not included in this repository.

## Install

### Windows PowerShell

```powershell
cd "C:\path\to\autonomous-driving-safety-simulator"
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

### macOS or Linux

```bash
cd /path/to/autonomous-driving-safety-simulator
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

## Run the core simulator

Windows:

```powershell
.\.venv\Scripts\python.exe -m autodrive_sim doctor
.\.venv\Scripts\python.exe -m autodrive_sim run --scenario .\scenarios\urban_crosswalk.toml --steps 100 --events .\artifacts\run.jsonl
```

macOS or Linux:

```bash
.venv/bin/python -m autodrive_sim doctor
.venv/bin/python -m autodrive_sim run --scenario scenarios/urban_crosswalk.toml --steps 100 --events artifacts/run.jsonl
```

The run prints a JSON summary. The optional JSONL file contains policy decisions, signal changes, collisions, and near misses.

## Run the live CARLA demo

Start CARLA first. Then install this project into the virtual environment that contains the matching CARLA Python package.

On Windows, the command looks like this:

```powershell
.\.carla-venv\Scripts\python.exe -m pip install -e ".[dev]"

.\.carla-venv\Scripts\python.exe -m autodrive_sim run `
  --backend carla `
  --scenario .\scenarios\carla_live_crossing.toml `
  --live `
  --compact-traffic `
  --traffic-manager `
  --duration-s 120
```

The demo runs in real time for two minutes. Press `Ctrl+C` to stop it sooner. Use `--duration-s 0` to keep it running until you stop it.

`--traffic-manager` enables CARLA's map-aware vehicle behavior. `--compact-traffic` keeps the vehicles close enough to see during a visual demo.

## Run the tests

```bash
python -m pytest -q
ruff check src tests
mypy
```

## Project layout

```text
src/autodrive_sim/   simulator code
tests/               automated tests
docs/                design notes and CARLA instructions
scenarios/           TOML scenario files
```

## License

This project is released under the MIT License. See [LICENSE](LICENSE).
