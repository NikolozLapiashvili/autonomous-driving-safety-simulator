from autodrive_sim.cli import build_parser, main


def test_cli_doctor_runs_without_carla(capsys) -> None:
    assert main(["doctor"]) == 0
    output = capsys.readouterr().out
    assert "Autonomy Traffic Lab" in output
    assert "Core simulator: ready" in output


def test_cli_runs_toml_scenario(tmp_path, capsys) -> None:
    scenario_path = tmp_path / "scenario.toml"
    scenario_path.write_text(
        "[scenario]\nname='cli-test'\nseed=2\nvehicle_count=1\npedestrian_count=0\n",
        encoding="utf-8",
    )

    assert main(["run", "--scenario", str(scenario_path), "--steps", "2"]) == 0
    assert '"scenario": "cli-test"' in capsys.readouterr().out


def test_cli_parses_realistic_carla_live_options() -> None:
    args = build_parser().parse_args(
        [
            "run",
            "--backend",
            "carla",
            "--live",
            "--compact-traffic",
            "--traffic-manager",
            "--duration-s",
            "0",
        ]
    )

    assert args.live
    assert args.compact_traffic
    assert args.traffic_manager
    assert args.duration_s == 0

