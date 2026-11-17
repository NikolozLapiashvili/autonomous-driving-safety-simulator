# CARLA 3D integration

The project treats CARLA as an optional runtime dependency. The Python safety core and deterministic backend can be developed and tested without a graphics card; CARLA supplies the real 3D map, actor meshes, physics, traffic lights, and sensors.

## Local setup

1. Download a CARLA release package for your operating system from the official CARLA release page.
2. Start the CARLA server locally.
3. Install the CARLA Python client that matches the server package. Do not mix client and server versions.
4. Run `autodrive-sim doctor` and confirm that the CARLA Python API is available.
5. Start with a small scenario: 5 vehicles and 2 pedestrians.

The exact client command is intentionally pinned only after the target OS and CARLA server release are chosen; CARLA distributes version-specific clients and server packages.

## Runtime contract

`CarlaBackend` configures synchronous mode and a fixed delta, translates domain actor poses into CARLA transforms, applies conservative throttle/brake commands, advances one server tick, and cleans up actors created by the run.

The first adapter uses CARLA ground-truth transforms and velocities. RGB camera, collision, and lane-invasion sensors are available behind the same domain snapshot boundary; LiDAR and learned perception can be added without changing the policy engine.

The sensor layer now provides front RGB recording, collision events, and lane-invasion events. RGB frames are written as `actor_rgb_<frame>.png` files under `artifacts/sensors/`; the other sensors use the same JSONL event sink as the policy engine.

## Practical expectations

- Use a discrete GPU for real-time 3D runs.
- Use synchronous mode for reproducible experiments.
- Keep actor counts modest until spawn placement and cleanup are validated.
- Record scenario seed, CARLA release, map, weather, and fixed delta in every experiment artifact.

## Live realistic traffic mode

The recommended visual run uses CARLA Traffic Manager instead of the small
manual controller. Traffic Manager uses map waypoints, collision hazards,
traffic-light hazards, junction priority, and a PID motion planner. The
adapter also holds pedestrians until the policy grants a walk phase and keeps
their destinations on the pedestrian navigation mesh.

```powershell
.\.carla-venv\Scripts\python.exe -m autodrive_sim run `
  --backend carla `
  --scenario .\scenarios\carla_live_crossing.toml `
  --live `
  --compact-traffic `
  --traffic-manager `
  --duration-s 120
```

Do not use the manual `--live` mode when evaluating actual junction behavior;
it exists for adapter development and low-level control experiments. The
Traffic Manager mode leaves CARLA's native traffic-light groups in control
instead of forcing every light in the map to the same abstract phase.

