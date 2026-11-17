# Roadmap

The roadmap is intentionally split into small deliverables so the history documents real engineering decisions.

## Foundation

- [x] package metadata, CLI, tests, and repository contract;
- [ ] domain entities and geometry primitives;
- [ ] deterministic simulation clock and backend;
- [ ] actor registry and spawn policies;
- [ ] configuration schema and scenario loader.

## Safety behavior

- [ ] traffic-light phase state machine;
- [ ] pedestrian crossing state machine;
- [ ] vehicle stopping and release policy;
- [ ] conflict-zone reservation;
- [ ] emergency and stale-observation handling;
- [ ] property-based safety tests.

## 3D integration

- [ ] CARLA connection and synchronous mode;
- [ ] vehicle and pedestrian spawning;
- [ ] traffic-light and crosswalk discovery;
- [ ] camera, LiDAR, collision, and lane-invasion sensors;
- [ ] actor cleanup and reconnect handling.

## Experimentation

- [ ] structured JSONL event logger;
- [ ] collision and near-miss metrics;
- [ ] scenario runner and batch sweeps;
- [ ] replay export;
- [ ] dashboard and recorded demo;
- [ ] GitHub Actions core CI and documentation.

