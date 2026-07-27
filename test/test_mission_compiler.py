from pathlib import Path

import numpy as np
import pytest
import yaml

from spot_navigation.mission_compiler import (
    BoundarySegments,
    CandidateResult,
    FarParameters,
    FreeDirection,
    MissionDocument,
    MissionPose,
    PriorGraph,
    VghNode,
    compile_mission,
    load_far_parameters,
    load_mission_document,
    load_vgh,
    render_compiled_preview,
)


IDENTITY_ORIENTATION = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "w": 1.0,
}


def _pose(x: float, y: float) -> MissionPose:
    return MissionPose(x, y, 0.0, 0.0, 0.0, 0.0, 1.0)


def _parameters(sensor_range: float = 10.0) -> FarParameters:
    return FarParameters(
        voxel_dim=0.1,
        robot_dim=0.2,
        vehicle_height=0.5,
        sensor_range=sensor_range,
        local_planner_range=2.0,
        angle_noise=np.deg2rad(10.0),
        converge_distance=0.75,
        floor_height=1.5,
        is_multi_layer=False,
    )


def _node(
    node_id: int,
    x: float,
    *,
    connections: tuple[int, ...] = (),
    free_direction: FreeDirection = FreeDirection.PILLAR,
) -> VghNode:
    return VghNode(
        node_id=node_id,
        free_direction=free_direction,
        position=np.asarray((x, 0.0, 0.75), dtype=np.float64),
        surface_directions=(
            np.asarray((1.0, 0.0, 0.0)),
            np.asarray((0.0, 1.0, 0.0)),
        ),
        is_covered=True,
        is_frontier=False,
        is_navpoint=False,
        is_boundary=False,
        connections=connections,
        polygon_connections=connections,
        contour_connections=(),
        trajectory_connections=(),
    )


def _mission(start: float, goal: float) -> MissionDocument:
    return MissionDocument("map", _pose(start, 0.0), (_pose(goal, 0.0),))


def test_boundary_segments_detect_crossing_and_allow_separated_line() -> None:
    boundaries = BoundarySegments(
        [
            (
                np.asarray((1.0, -1.0, 0.75)),
                np.asarray((1.0, 1.0, 0.75)),
            )
        ]
    )

    assert boundaries.intersects(
        np.asarray((0.0, 0.0)),
        np.asarray((2.0, 0.0)),
    )
    assert not boundaries.intersects(
        np.asarray((0.0, 2.0)), np.asarray((2.0, 2.0))
    )


def test_compile_direct_leg_uses_synthetic_start_and_densifies() -> None:
    graph = PriorGraph({0: _node(0, 0.0)}, _parameters())

    compiled, report = compile_mission(
        _mission(0.0, 5.0),
        graph,
        (np.asarray((-1.0, -1.0, -1.0)), np.asarray((6.0, 1.0, 1.0))),
        max_goal_spacing=2.0,
    )

    assert report["status"] == "VALIDATED_STATIC"
    assert report["inserted_waypoint_count"] == 2
    assert report["compiled_waypoint_count"] == 3
    assert report["legs"][0]["mode"] == "direct"
    compiled_x = [
        pose["position"]["x"] for pose in compiled["waypoints"]
    ]
    assert compiled_x == pytest.approx([5.0 / 3.0, 10.0 / 3.0, 5.0])


def test_render_compiled_preview_distinguishes_inserted_goals(
    tmp_path: Path,
) -> None:
    graph = PriorGraph({0: _node(0, 0.0)}, _parameters())
    mission = _mission(0.0, 5.0)
    compiled, report = compile_mission(
        mission,
        graph,
        (np.asarray((-1.0, -1.0, -1.0)), np.asarray((6.0, 1.0, 1.0))),
        max_goal_spacing=2.0,
    )
    pcd_points = np.asarray(
        [
            [-1.0, -1.0, 0.0],
            [-1.0, 1.0, 0.0],
            [6.0, -1.0, 0.0],
            [6.0, 1.0, 0.0],
        ],
        dtype=np.float32,
    )
    output_path = tmp_path / "compiled.png"

    render_compiled_preview(
        pcd_points,
        output_path,
        mission,
        compiled,
        report,
        graph,
    )

    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_path.stat().st_size > 10_000


def test_graph_route_is_used_when_boundary_blocks_direct_leg() -> None:
    nodes = {
        0: VghNode(
            **{
                **_node(0, 0.0, connections=(1,)).__dict__,
                "position": np.asarray((0.0, 2.0, 0.75)),
            }
        ),
        1: VghNode(
            **{
                **_node(1, 4.0, connections=(0,)).__dict__,
                "position": np.asarray((4.0, 2.0, 0.75)),
            }
        ),
        2: VghNode(
            node_id=2,
            free_direction=FreeDirection.PILLAR,
            position=np.asarray((2.0, 0.0, 0.75)),
            surface_directions=(
                np.asarray((1.0, 0.0, 0.0)),
                np.asarray((0.0, 1.0, 0.0)),
            ),
            is_covered=True,
            is_frontier=False,
            is_navpoint=False,
            is_boundary=True,
            connections=(),
            polygon_connections=(),
            contour_connections=(3,),
            trajectory_connections=(),
        ),
        3: VghNode(
            node_id=3,
            free_direction=FreeDirection.PILLAR,
            position=np.asarray((2.0, 1.0, 0.75)),
            surface_directions=(
                np.asarray((1.0, 0.0, 0.0)),
                np.asarray((0.0, 1.0, 0.0)),
            ),
            is_covered=True,
            is_frontier=False,
            is_navpoint=False,
            is_boundary=True,
            connections=(),
            polygon_connections=(),
            contour_connections=(2,),
            trajectory_connections=(),
        ),
    }
    graph = PriorGraph(nodes, _parameters())

    plan = graph.plan_leg(_pose(0.0, 0.0), _pose(4.0, 0.0))

    assert plan is not None
    assert plan.mode == "graph"
    assert plan.graph_path == [0, 1]
    assert len(plan.route) == 4


def test_concave_start_candidate_is_reported_by_reason() -> None:
    graph = PriorGraph(
        {
            0: _node(
                0,
                1.0,
                free_direction=FreeDirection.CONCAVE,
            )
        },
        _parameters(),
    )

    candidates = graph._start_candidates(np.asarray((0.0, 0.0, 0.75)))

    assert candidates == CandidateResult([], {"concave": [0]})


def test_compile_rejects_pose_outside_pcd_bounds() -> None:
    graph = PriorGraph({0: _node(0, 0.0)}, _parameters())

    with pytest.raises(ValueError, match="outside the PCD XY bounds"):
        compile_mission(
            _mission(0.0, 5.0),
            graph,
            (np.asarray((-1.0, -1.0, -1.0)), np.asarray((4.0, 1.0, 1.0))),
            max_goal_spacing=2.0,
        )


def test_load_vgh_rejects_asymmetric_connections(tmp_path: Path) -> None:
    path = tmp_path / "bad.vgh"
    path.write_text(
        "\n".join(
            [
                "0 3 0 0 0.75 1 0 0 0 1 0 1 0 0 0 1 | 1 | |",
                "1 3 1 0 0.75 1 0 0 0 1 0 1 0 0 0 | 0 | |",
            ]
        )
        + "\n",
        encoding="ascii",
    )

    with pytest.raises(ValueError, match="asymmetric"):
        load_vgh(path)


def test_load_inputs_match_route_manager_schema(tmp_path: Path) -> None:
    mission_path = tmp_path / "mission.yaml"
    mission_path.write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "initial_pose": {
                    "position": {"x": 0, "y": 0, "z": 0},
                    "orientation": IDENTITY_ORIENTATION,
                },
                "waypoints": [
                    {
                        "position": {"x": 1, "y": 2, "z": 0},
                        "orientation": IDENTITY_ORIENTATION,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    config_path = tmp_path / "far.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "far_planner": {
                    "ros__parameters": {
                        "voxel_dim": 0.1,
                        "robot_dim": 0.2,
                        "vehicle_height": 0.5,
                        "sensor_range": 10.0,
                        "local_planner_range": 2.0,
                        "is_multi_layer": False,
                        "util/angle_noise": 10.0,
                        "g_planner/converge_distance": 0.75,
                        "map_handler/floor_height": 1.5,
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    mission = load_mission_document(mission_path)
    parameters = load_far_parameters(config_path)

    assert mission.frame_id == "map"
    assert mission.waypoints[0].x == pytest.approx(1.0)
    assert parameters.nav_clear_distance == pytest.approx(0.2)
    assert parameters.angle_noise == pytest.approx(np.deg2rad(10.0))
