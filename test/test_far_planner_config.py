from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RELAUNCH_TUNING_PARAMETERS = [
    "sensor_range",
    "terrain_range",
    "local_planner_range",
    "util/obs_inflate_size",
    "g_planner/converge_distance",
    "g_planner/reach_goal_vote_size",
]


def test_far_config_starts_with_relaunch_tuning_parameters() -> None:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "far_planner.yaml").read_text(
            encoding="utf-8"
        )
    )
    parameters = config["far_planner"]["ros__parameters"]

    assert list(parameters)[:6] == RELAUNCH_TUNING_PARAMETERS


def test_readme_documents_yaml_relaunch_tuning() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    for parameter_name in RELAUNCH_TUNING_PARAMETERS:
        assert f"`{parameter_name}`" in readme

    assert "ros2 param set /far_planner" not in readme
    assert "relaunch the\nsame mission" in readme
