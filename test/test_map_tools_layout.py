from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
MAP_TOOLS = PACKAGE_ROOT / "scripts" / "map_tools"


def test_offline_map_internals_are_colocated_under_scripts() -> None:
    expected_modules = {
        "__init__.py",
        "alignment.py",
        "pointcloud.py",
        "ground.py",
        "interaction.py",
        "far_grid.py",
        "far_contours.py",
        "visibility_graph.py",
        "outputs.py",
    }

    assert {path.name for path in MAP_TOOLS.glob("*.py")} == expected_modules


def test_map_tool_modules_are_installed_with_entry_scripts() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert '"share/" + package_name + "/scripts/map_tools"' in setup_source
    assert 'glob("scripts/map_tools/*.py")' in setup_source
