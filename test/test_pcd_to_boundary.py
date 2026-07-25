from pathlib import Path

import numpy as np
import pytest

from spot_navigation.pcd_to_boundary import (
    FreeDirection,
    build_visibility_graph,
    choose_free_point,
    extract_boundary_polygons,
    point_inside_polygon,
    select_free_space_component,
    segments_intersect,
    shoelace_area,
    write_vgh,
)


def _two_obstacle_polygons() -> list[np.ndarray]:
    return [
        np.asarray(
            [
                [-4.0, -1.0],
                [-2.0, -1.0],
                [-2.0, 1.0],
                [-4.0, 1.0],
            ],
            dtype=np.float32,
        ),
        np.asarray(
            [
                [2.0, -1.0],
                [4.0, -1.0],
                [4.0, 1.0],
                [2.0, 1.0],
            ],
            dtype=np.float32,
        ),
    ]


def test_segments_intersect_matches_boundary_handler_cases() -> None:
    assert segments_intersect(
        np.asarray([0.0, 0.0]),
        np.asarray([2.0, 2.0]),
        np.asarray([0.0, 2.0]),
        np.asarray([2.0, 0.0]),
    )
    assert segments_intersect(
        np.asarray([0.0, 0.0]),
        np.asarray([1.0, 0.0]),
        np.asarray([1.0, 0.0]),
        np.asarray([2.0, 0.0]),
    )
    assert not segments_intersect(
        np.asarray([0.0, 0.0]),
        np.asarray([1.0, 0.0]),
        np.asarray([2.0, 0.0]),
        np.asarray([3.0, 0.0]),
    )


def test_visibility_graph_matches_boundary_handler_connectivity() -> None:
    graph = build_visibility_graph(
        _two_obstacle_polygons(),
        (-0.0, 0.0, 0.75),
        0.75,
    )

    assert [node.free_direction for node in graph] == [FreeDirection.CONVEX] * 8
    assert [node.connections for node in graph] == [
        [1, 3],
        [0, 2, 7],
        [1, 3, 4],
        [0, 2],
        [2, 5, 7],
        [4, 6],
        [5, 7],
        [1, 4, 6],
    ]
    assert [node.contour_connections for node in graph] == [
        [1, 3],
        [0, 2],
        [1, 3],
        [2, 0],
        [5, 7],
        [4, 6],
        [5, 7],
        [6, 4],
    ]


def test_write_vgh_matches_boundary_handler_format(tmp_path: Path) -> None:
    graph = build_visibility_graph(
        _two_obstacle_polygons(),
        (0.0, 0.0, 0.75),
        0.75,
    )
    output_path = tmp_path / "map.vgh"

    write_vgh(output_path, graph)

    lines = output_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 8
    assert lines[0] == (
        "0 1 -4.000000 -1.000000 0.750000 "
        "0.000000 1.000000 0.000000 "
        "1.000000 0.000000 0.000000 "
        "1 0 0 1 1 3 | 1 3 | 1 3 | "
    )
    assert lines[1].endswith("0 2 7 | 0 2 7 | 0 2 | ")
    assert all(line.count("|") == 3 for line in lines)


def test_write_vgh_rejects_empty_graph(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="empty visibility graph"):
        write_vgh(tmp_path / "empty.vgh", [])


def test_enclosed_map_keeps_inner_obstacle_boundaries() -> None:
    occupied_cells: set[tuple[int, int]] = set()
    for coordinate in range(21):
        occupied_cells.add((coordinate, 0))
        occupied_cells.add((coordinate, 20))
        occupied_cells.add((0, coordinate))
        occupied_cells.add((20, coordinate))
    for x_coordinate in range(8, 13):
        for y_coordinate in range(8, 13):
            occupied_cells.add((x_coordinate, y_coordinate))

    points = np.asarray(
        [
            [float(x_coordinate), float(y_coordinate), 1.0]
            for x_coordinate, y_coordinate in sorted(occupied_cells)
        ],
        dtype=np.float32,
    )
    polygons, stats, occupancy, origin, _ = extract_boundary_polygons(
        points,
        points,
        1.0,
        1.0,
        0.0,
        0.0,
        0.1,
        1.0,
        10,
        80,
        True,
        None,
        0.0,
    )
    free_point = choose_free_point(occupancy, origin, 1.0, 0.75)
    polygon_areas = sorted(abs(shoelace_area(polygon)) for polygon in polygons)

    assert 1.0 < free_point[0] < 19.0
    assert 1.0 < free_point[1] < 19.0
    assert len(polygons) == 2
    assert stats["obstacle_polygons"] == 1
    assert polygon_areas[0] < 50.0
    assert point_inside_polygon(polygons[-1], np.asarray(free_point)) is False


def test_requested_free_point_selects_its_connected_component() -> None:
    occupancy = np.zeros((5, 8), dtype=np.uint8)
    occupancy[:, 5] = 255

    component, stats = select_free_space_component(
        occupancy,
        np.asarray([0.0, 0.0]),
        1.0,
        np.asarray([6.5, 2.5]),
    )

    assert stats["selection"] == "requested"
    assert stats["selected_cells"] == 10
    assert component[2, 6] == 255
    assert component[2, 1] == 0
