"""Independent geometric checks for Task8 candidate generation, with no ROS."""

import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from scene_route import load_scene  # noqa: E402
from task8_reverse_geometry import reverse_real  # noqa: E402


@pytest.fixture
def routes():
    forward = load_scene(ROOT / 'waypoints/waypoints_real.yaml', 2)
    return forward, reverse_real(forward)


def test_independently_calibrated_reverse_is_preserved():
    import hashlib

    path = ROOT / 'waypoints/reverse_waypoints_real.yaml'
    assert (
        hashlib.sha256(path.read_bytes()).hexdigest()
        == '3e841c52243fb16c378244132fa657a42c4fb04fa5984b2323206717e3fea064'
    )
    checked = load_scene(path, 4)
    assert len(checked['actions']) == 21
    paths = [a for a in checked['actions'] if a['kind'] == 'path']
    for left, right in zip(paths, paths[1:]):
        assert left['points'][-1] == pytest.approx(right['points'][0])
    for action in paths:
        assert all(
            math.dist(x, y) <= 0.05 + 1e-9 for x, y in zip(action['points'], action['points'][1:])
        )


def test_full_waypoint_order_and_explicit_positive_terminal(routes):
    _, reverse = routes
    paths = [a for a in reverse['actions'] if a['kind'] == 'path']
    assert [a['name'] for a in paths] == [f'P{i:02d}' for i in range(14, 0, -1)]
    terminal = reverse['actions'][-1]
    assert terminal['name'] == 'P01_after_turn'
    assert terminal['heading_rad'] - paths[-1]['heading_rad'] == pytest.approx(math.pi)
    assert math.cos(terminal['heading_rad']) == pytest.approx(1.0)
    assert reverse['validated'] is False


def test_every_interpolated_point_and_heading_retraces_physical_route(routes):
    forward, reverse = routes
    original = [a for a in forward['actions'] if a['kind'] == 'path']
    inbound = [a for a in reverse['actions'] if a['kind'] == 'path']
    endpoint = original[-1]['points'][-1]
    for outward, back in zip(reversed(original), inbound):
        assert len(back['points']) == len(outward['points'])
        for old, new in zip(reversed(outward['points']), back['points']):
            assert [new[0] + endpoint[0], new[1] + endpoint[1]] == pytest.approx(old)
        assert math.cos(back['heading_rad'] - outward['heading_rad']) == pytest.approx(-1.0)
    assert inbound[0]['points'][0] == pytest.approx([0.0, 0.0])
    assert inbound[-1]['points'][-1] == pytest.approx([-endpoint[0], -endpoint[1]])


def test_body_motion_is_recomputed_not_blindly_negated(routes):
    forward, reverse = routes
    original = [a for a in forward['actions'] if a['kind'] == 'path']
    inbound = [a for a in reverse['actions'] if a['kind'] == 'path']

    def body_delta(action):
        start, end = action['points'][0], action['points'][-1]
        dx, dy = end[0] - start[0], end[1] - start[1]
        c, s = math.cos(action['heading_rad']), math.sin(action['heading_rad'])
        return c * dx + s * dy, -s * dx + c * dy

    for outward, back in zip(reversed(original), inbound):
        assert body_delta(back) == pytest.approx(body_delta(outward))


def test_legacy_route_without_terminal_is_rejected(routes):
    forward, _ = routes
    forward['actions'].pop()
    with pytest.raises(ValueError, match='complete outbound'):
        reverse_real(forward)


def test_generated_scene_four_loads_as_unvalidated_candidate(routes, tmp_path):
    _, reverse = routes
    import yaml

    candidate = tmp_path / 'reverse.yaml'
    candidate.write_text(yaml.safe_dump(reverse))
    loaded = load_scene(candidate, 4)
    assert loaded['validated'] is False
    assert loaded['actions'][-1]['name'] == 'P01_after_turn'
