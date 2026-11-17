from autodrive_sim.cli import main


def test_cli_doctor_runs_without_carla(capsys) -> None:
    assert main(["doctor"]) == 0
    output = capsys.readouterr().out
    assert "Autonomy Traffic Lab" in output
    assert "Core simulator: ready" in output

