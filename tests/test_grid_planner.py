"""Tests for the grid planner in manhattan_graph_dijkstra.py (no robot needed).

Grid coordinates are screen-style: (0, 0) is the top-left corner and y grows
downwards, so facing east (1, 0) and moving to y + 1 is a right turn.

Run from the repository root with: python -m pytest
"""
import random
from collections import deque

import pytest

from manhattan_graph_dijkstra import Grid


def bfs_length(grid, start, end):
    """Number of nodes on a shortest path, or 0 if unreachable (reference answer)."""
    seen, queue = {start: 1}, deque([start])
    while queue:
        node = queue.popleft()
        if node is end:
            return seen[node]
        for nxt in node.neighbors:
            if nxt not in seen:
                seen[nxt] = seen[node] + 1
                queue.append(nxt)
    return 0


def assert_valid_path(grid, path, start, end):
    assert path[0] is start and path[-1] is end
    for a, b in zip(path, path[1:]):
        assert b in a.neighbors


def test_grid_is_fully_connected_lattice():
    grid = Grid(4, 3)
    assert len(grid.nodes) == 12
    degrees = sorted(len(n.neighbors) for n in grid.nodes)
    assert degrees.count(2) == 4  # corners
    assert degrees.count(4) == 2  # interior
    assert all(w == 1 for w in grid.edgeToWeight.values())


def test_shortest_paths_on_full_grid():
    grid = Grid(4, 4)
    for start in grid.nodes:
        for end in grid.nodes:
            path = grid.dijkstra(start, end)
            assert_valid_path(grid, path, start, end)
            manhattan = abs(start.pos[0] - end.pos[0]) + abs(start.pos[1] - end.pos[1])
            assert len(path) == manhattan + 1


@pytest.mark.parametrize("seed", range(5))
def test_shortest_paths_with_blocked_edges(seed):
    rng = random.Random(seed)
    grid = Grid(5, 4)
    edges = sorted({tuple(sorted((a.pos, b.pos))) for a in grid.nodes for b in a.neighbors})
    for a, b in rng.sample(edges, 8):
        grid.removeEdge(grid.posToNode[a], grid.posToNode[b])

    for start in grid.nodes:
        for end in grid.nodes:
            path = grid.dijkstra(start, end)
            expected = bfs_length(grid, start, end)
            if expected == 0:
                assert path == []
            else:
                assert_valid_path(grid, path, start, end)
                assert len(path) == expected


def test_removed_node_is_avoided():
    grid = Grid(3, 3)
    middle = grid.posToNode[(1, 1)]
    grid.removeNode(middle)
    path = grid.dijkstra(grid.posToNode[(1, 0)], grid.posToNode[(1, 2)])
    assert middle not in path
    assert len(path) == 5  # around the side instead of 3 through the middle
    assert not grid.isInGraph((1, 1))


def test_unreachable_end_returns_empty_path():
    grid = Grid(3, 1)
    grid.removeEdge(grid.posToNode[(0, 0)], grid.posToNode[(1, 0)])
    assert grid.dijkstra(grid.posToNode[(0, 0)], grid.posToNode[(2, 0)]) == []


def test_multi_path_goes_through_pitstops_in_order():
    grid = Grid(3, 3)
    stops = [grid.posToNode[p] for p in [(0, 0), (2, 0), (2, 2)]]
    path = grid.multi_path_dijkstra(stops)
    assert_valid_path(grid, path, stops[0], stops[-1])
    assert path.index(stops[1]) < len(path) - 1
    assert len(path) == 5  # junction node not repeated


@pytest.mark.parametrize("move, expected", [
    ((1, 0), "straight"),
    ((0, 1), "right"),
    ((0, -1), "left"),
    ((-1, 0), "backwards"),
])
def test_get_dir_facing_east(move, expected):
    grid = Grid(3, 3)
    start = (1, 1)
    end = (start[0] + move[0], start[1] + move[1])
    assert grid.getDir(start, end, 1, 0) == expected


def test_get_dir_rejects_bad_input():
    grid = Grid(3, 3)
    with pytest.raises(ValueError):
        grid.getDir((0, 0), (2, 0), 1, 0)  # not adjacent
    with pytest.raises(ValueError):
        grid.getDir((0, 0), (1, 0), 1, 1)  # not a cardinal direction


def test_path_to_directions():
    grid = Grid(3, 3)
    path = [grid.posToNode[p] for p in [(0, 0), (1, 0), (1, 1), (1, 2), (2, 2)]]
    assert grid.convertNodePathToDirections(path) == ["right", "straight", "left", "end"]
    assert grid.convertNodePathToDirections(path[:1]) == []


def test_dir3nodes():
    grid = Grid(3, 3)
    a, b, c = (grid.posToNode[p] for p in [(0, 1), (1, 1), (1, 0)])
    assert grid.dir3Nodes(a, b, c) == "left"  # heading east, then up the screen
    with pytest.raises(ValueError):
        grid.dir3Nodes(a, c, b)


def test_is_valid_pos():
    grid = Grid(3, 2)
    assert grid.isValidPos((2, 1))
    assert not grid.isValidPos((3, 0))
    grid.illegal_positions.add((1, 1))
    assert not grid.isValidPos((1, 1))
