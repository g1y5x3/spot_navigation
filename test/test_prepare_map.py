import importlib.util
from pathlib import Path
import subprocess
import sys


SCRIPT_PATH = Path(__file__).parents[1] / "scripts/prepare_map.py"
SPEC = importlib.util.spec_from_file_location("prepare_map", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
prepare_map = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare_map)


def test_dry_run_keeps_both_stage_commands_visible(tmp_path: Path) -> None:
    raw_pcd = tmp_path / "raw map.pcd"
    raw_pcd.touch()
    output_dir = tmp_path / "prepared"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(raw_pcd),
            "--output-dir",
            str(output_dir),
            "--name",
            "dorsett",
            "--voxel-size",
            "0.07",
            "--statistical-neighbors",
            "24",
            "--statistical-std-ratio",
            "1.8",
            "--radius-outlier-neighbors",
            "4",
            "--radius-outlier-radius",
            "0.35",
            "--no-show-3d-qc",
            "--dry-run",
            "--",
            "--resolution",
            "0.20",
            "--free-point",
            "1.0",
            "2.0",
            "0.75",
        ],
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "[1/2] Align point cloud" in result.stdout
    assert "align_pointcloud_to_map_frame.py" in result.stdout
    assert "--voxel-size 0.07" in result.stdout
    assert "--statistical-neighbors 24" in result.stdout
    assert "--statistical-std-ratio 1.8" in result.stdout
    assert "--radius-outlier-neighbors 4" in result.stdout
    assert "--radius-outlier-radius 0.35" in result.stdout
    assert "--no-show-3d-qc" in result.stdout
    assert "[2/2] Build FAR prior map" in result.stdout
    assert "build_far_prior_map.py" in result.stdout
    assert "--resolution 0.20 --free-point 1.0 2.0 0.75" in result.stdout
    assert str(output_dir / "dorsett_transformed.pcd") in result.stdout
    assert str(output_dir / "dorsett_transformed.vgh") in result.stdout
    assert not output_dir.exists()


def test_dry_run_omits_disabled_preview_from_planned_outputs(
    tmp_path: Path,
) -> None:
    raw_pcd = tmp_path / "raw.pcd"
    raw_pcd.touch()
    output_dir = tmp_path / "prepared"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(raw_pcd),
            "--output-dir",
            str(output_dir),
            "--dry-run",
            "--",
            "--no-preview",
        ],
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    planned_outputs = result.stdout.split("Outputs:\n", 1)[1]
    assert "raw_transformed_boundary_stats.json" in planned_outputs
    assert "raw_transformed_boundary_preview.png" not in planned_outputs


def test_pipeline_runs_both_scripts_and_names_alignment_outputs(
    tmp_path: Path,
    monkeypatch,
) -> None:
    script_dir = tmp_path / "scripts"
    script_dir.mkdir()
    (script_dir / "align_pointcloud_to_map_frame.py").write_text(
        """from pathlib import Path
import sys
Path('align_args.txt').write_text(' '.join(sys.argv[1:]), encoding='utf-8')
Path('transformed.pcd').write_text('aligned', encoding='utf-8')
Path('estimated_transform.json').write_text('{}', encoding='utf-8')
Path('transform_debug.png').write_text('preview', encoding='utf-8')
""",
        encoding="utf-8",
    )
    (script_dir / "build_far_prior_map.py").write_text(
        """from pathlib import Path
import sys
args = sys.argv[1:]
aligned_pcd = Path(args[0])
assert aligned_pcd.read_text(encoding='utf-8') == 'aligned'
output_dir = Path(args[args.index('--output-dir') + 1])
name = args[args.index('--name') + 1]
(output_dir / 'build_args.txt').write_text(' '.join(args), encoding='utf-8')
(output_dir / f'{name}.vgh').write_text('graph', encoding='utf-8')
""",
        encoding="utf-8",
    )
    raw_pcd = tmp_path / "raw.pcd"
    raw_pcd.touch()
    output_dir = tmp_path / "prepared"
    output_dir.mkdir()
    original_pcd = output_dir / "dorsett.pcd"
    original_pcd.write_text("original", encoding="utf-8")
    monkeypatch.setattr(prepare_map, "SCRIPT_DIR", script_dir)

    return_code = prepare_map.main(
        [
            str(raw_pcd),
            "--output-dir",
            str(output_dir),
            "--name",
            "dorsett",
            "--voxel-size",
            "0.06",
            "--no-noise-filter",
            "--no-show-3d-qc",
            "--",
            "--resolution",
            "0.20",
        ]
    )

    assert return_code == 0
    assert (
        output_dir / "dorsett_transformed.pcd"
    ).read_text(encoding="utf-8") == "aligned"
    assert original_pcd.read_text(encoding="utf-8") == "original"
    assert (output_dir / "dorsett_transform.json").is_file()
    assert (output_dir / "dorsett_transform_debug.png").is_file()
    assert not (output_dir / "transformed.pcd").exists()
    assert (
        output_dir / "dorsett_transformed.vgh"
    ).read_text(encoding="utf-8") == "graph"
    assert "--voxel-size 0.06" in (
        output_dir / "align_args.txt"
    ).read_text(encoding="utf-8")
    assert "--no-noise-filter" in (
        output_dir / "align_args.txt"
    ).read_text(encoding="utf-8")
    assert "--no-show-3d-qc" in (
        output_dir / "align_args.txt"
    ).read_text(encoding="utf-8")
    assert "--resolution 0.20" in (
        output_dir / "build_args.txt"
    ).read_text(encoding="utf-8")
    assert "--name dorsett_transformed" in (
        output_dir / "build_args.txt"
    ).read_text(encoding="utf-8")
