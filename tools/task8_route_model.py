"""Strict scene-4 manifests and directed terminal motion; no ROS side effects."""

import math

from scene_route import heading_equal, mapping, validate_action

TURN_POINTS = {14, 13, 12, 11, 10, 3}
ACTION_NAMES = tuple(
    name
    for i in range(14, 0, -1)
    for name in ([f'P{i:02d}', f'P{i:02d}_after_turn'] if i in TURN_POINTS else [f'P{i:02d}'])
) + ('P01_after_turn',)
TERMINAL_NAME = 'P01_after_turn'


def validate_reverse(data):
    mapping(data, 'scene4')
    if type(data.get('validated')) is not bool:
        raise ValueError('Scene4 requires explicit validated boolean')
    actions = data.get('actions')
    if (
        not isinstance(actions, list)
        or tuple(a.get('name') for a in actions if isinstance(a, dict)) != ACTION_NAMES
    ):
        raise ValueError('Scene4 requires complete P14..P01 action order and terminal turn')
    position, heading, result = [0.0, 0.0], 0.0, []
    for action in actions:
        expected = 'turn' if action['name'].endswith('_after_turn') else 'path'
        if action.get('kind') != expected:
            raise ValueError('Scene4 labels must match action kind')
        clean, position, heading = validate_action(action, position, heading)
        if (
            clean['kind'] == 'path'
            and sum(math.dist(a, b) for a, b in zip(clean['points'], clean['points'][1:]))
            > 2.0 + 1e-9
        ):
            raise ValueError('Scene4 path must be at most 2 m')
        result.append(clean)
    if not heading_equal(result[-1]['heading_rad'] - result[-2]['heading_rad'], math.pi):
        raise ValueError('P01_after_turn must follow P01 by a half-turn')
    if result[-1]['heading_rad'] - result[-2]['heading_rad'] != math.pi:
        raise ValueError('P01_after_turn must declare signed +pi')
    if data.get('test_until') not in ACTION_NAMES:
        raise ValueError('Scene4 requires declared test_until')
    return dict(
        schema=1,
        scene=4,
        frame='P15',
        validated=data['validated'],
        test_until=data['test_until'],
        actions=result,
    )


def select_reverse(route, destination='P01', execute=False):
    label = TERMINAL_NAME if destination == 'P01' else destination
    if label not in ACTION_NAMES:
        raise ValueError('Unknown reverse destination')
    index = ACTION_NAMES.index(label)
    if execute and not route['validated'] and index > ACTION_NAMES.index(route['test_until']):
        raise ValueError('Reverse route is not validated beyond test_until')
    return dict(schema=1, scene=4, frame='P15', actions=route['actions'][: index + 1])


def validate_manifest(data):
    if not isinstance(data, dict) or set(data) != {'schema', 'scene', 'frame', 'actions'}:
        raise ValueError('Expected schema, scene, frame and actions only')
    if (
        type(data['schema']) is not int
        or data['schema'] != 1
        or type(data['scene']) is not int
        or data['scene'] != 4
        or data['frame'] != 'P15'
    ):
        raise ValueError('Expected schema1 scene4 P15 frame')
    raw = data['actions']
    if not isinstance(raw, list) or not raw or len(raw) > len(ACTION_NAMES):
        raise ValueError('Expected nonempty scene4 prefix')
    if tuple(a.get('name') for a in raw if isinstance(a, dict)) != ACTION_NAMES[: len(raw)]:
        raise ValueError('Scene4 payload must preserve exact prefix order')
    # Validate the prefix independently without inventing the omitted tail.
    position, heading, result = [0.0, 0.0], 0.0, []
    from task6_route_model import RouteAction

    for action in raw:
        expected = 'turn' if action['name'].endswith('_after_turn') else 'path'
        if action.get('kind') != expected:
            raise ValueError('Scene4 label and kind mismatch')
        clean, position, heading = validate_action(action, position, heading)
        if (
            clean['kind'] == 'path'
            and sum(math.dist(a, b) for a, b in zip(clean['points'], clean['points'][1:]))
            > 2.0 + 1e-9
        ):
            raise ValueError('Scene4 path must be at most 2 m')
        result.append(
            RouteAction(
                clean['name'],
                clean['kind'],
                clean['heading_rad'],
                tuple(tuple(p) for p in clean.get('points', ())),
            )
        )
    if (
        result[-1].name == TERMINAL_NAME
        and result[-1].heading_rad - result[-2].heading_rad != math.pi
    ):
        raise ValueError('P01 terminal must be signed +pi')
    return tuple(result)
