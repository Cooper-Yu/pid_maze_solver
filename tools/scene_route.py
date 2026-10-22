"""Strict simulation scene files and fixed-P01 real-route validation; no ROS or motion side effects."""

import math
from pathlib import Path

import yaml

STEP_NAMES = tuple(f'P{i:02d}_P{i + 1:02d}' for i in range(1, 15))
REVERSE_STEP_NAMES = tuple(f'P{i:02d}_P{i - 1:02d}' for i in range(15, 1, -1))
SCENE_FRAMES = {1: 'P01', 2: 'P01', 3: 'P15'}
TURN_AFTER = {3, 10, 11, 12, 13, 14}
ACTION_NAMES = tuple(
    name
    for i in range(2, 16)
    for name in ([f'P{i:02d}', f'P{i:02d}_after_turn'] if i in TURN_AFTER else [f'P{i:02d}'])
)
TERMINAL_ACTION = 'P15_after_turn'
COMPLETE_ACTION_NAMES = ACTION_NAMES + (TERMINAL_ACTION,)


class UniqueLoader(yaml.SafeLoader):
    """Reject ambiguous duplicate YAML keys rather than silently taking the last value."""


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ValueError('YAML keys must be unique strings')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label} must be finite numeric')
    return float(value)


def mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def xy(value):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError('Every path point must be an XY pair')
    return [number(v, 'path coordinate') for v in value]


def heading_equal(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) < 1e-9


def validate_step(name, step):
    mapping(step, name)
    required = {'turn_deg', 'forward_m', 'left_m', 'max_speed'}
    if not required <= step.keys() or step.keys() - required - {'side_centering'}:
        raise ValueError(f'{name}: expected complete named motion step')
    result = {key: number(step[key], f'{name}.{key}') for key in required}
    if abs(result['turn_deg']) > 180 or result['max_speed'] <= 0:
        raise ValueError(f'{name}: invalid turn or speed')
    if 'side_centering' in step:
        if type(step['side_centering']) is not bool:
            raise ValueError(f'{name}: side_centering must be boolean')
        result['side_centering'] = step['side_centering']
    return result


def validate_sim(data, scene=1):
    if scene not in (1, 3):
        raise ValueError('Unsupported simulation scene')
    params = mapping(data.get('parameters'), 'parameters')
    if params.get('route_mode') != 'motion_steps':
        raise ValueError(f'Scene {scene} file must preserve the motion_steps policy')
    allowed = {
        'route_mode',
        'steps',
        'final_clockwise_turn',
        'side_centering',
        'max_speed',
        'max_acceleration',
        'stop_hold',
        'stage_timeout',
        'max_yaw_rate',
        'turn_stop_yaw_rate',
        'distance_kp',
        'distance_ki',
        'distance_kd',
        'turn_kp',
        'turn_ki',
        'turn_kd',
    }
    if scene == 3:
        allowed.update(('route_direction', 'final_counterclockwise_turn'))
        if params.get('route_direction') != 'reverse':
            raise ValueError('Scene 3 requires route_direction: reverse')
    if params.keys() - allowed:
        raise ValueError(f'Unknown scene {scene} parameter')
    for key, value in params.items():
        if key in ('final_clockwise_turn', 'final_counterclockwise_turn', 'side_centering'):
            if type(value) is not bool:
                raise ValueError(f'{key} must be boolean')
        elif key not in ('route_mode', 'steps', 'route_direction'):
            v = number(value, key)
            if v < 0 or (not key.endswith(('_ki', '_kd')) and v == 0):
                raise ValueError(f'{key} must be positive (I/D may be zero)')
    if params.get('final_counterclockwise_turn', False) and params.get(
        'final_clockwise_turn', True
    ):
        raise ValueError('Terminal turn directions are mutually exclusive')
    steps = mapping(params.get('steps'), 'steps')
    names = STEP_NAMES if scene == 1 else REVERSE_STEP_NAMES
    if set(steps) != set(names):
        raise ValueError(f'Scene {scene} requires the complete ordered step names')
    params = dict(params, steps={name: validate_step(name, steps[name]) for name in names})
    return dict(schema=1, scene=scene, frame=SCENE_FRAMES[scene], parameters=params)


def validate_action(action, position, heading):
    mapping(action, 'action')
    kind = action.get('kind')
    keys = {'name', 'kind', 'heading_rad'} | ({'points'} if kind == 'path' else set())
    if kind not in ('path', 'turn') or set(action) != keys:
        raise ValueError('Action requires name/kind/heading_rad and path points only')
    yaw = number(action['heading_rad'], 'heading_rad')
    if abs(yaw) > 4 * math.pi:
        raise ValueError('Heading outside supported unwrapped range')
    result = dict(name=action['name'], kind=kind, heading_rad=yaw)
    if kind == 'turn':
        return result, position, yaw
    if not heading_equal(heading, yaw):
        raise ValueError('Path heading changed without an explicit turn')
    points = action['points']
    if not isinstance(points, list) or not 2 <= len(points) <= 400:
        raise ValueError('Path requires 2..400 fixed-P01 points')
    points = [xy(point) for point in points]
    if math.dist(points[0], position) > 1e-9:
        raise ValueError('Discontinuous path start or moved P01 origin')
    if any(not 0 < math.dist(a, b) <= 0.050000001 for a, b in zip(points, points[1:])):
        raise ValueError('Path edges must be nonzero and at most 5 cm')
    result['points'] = points
    return result, points[-1], yaw


def validate_real(data):
    if type(data.get('validated')) is not bool:
        raise ValueError('Scene 2 requires explicit validated boolean')
    actions = data.get('actions')
    if not isinstance(actions, list) or tuple(
        a.get('name') for a in actions if isinstance(a, dict)
    ) not in (ACTION_NAMES, COMPLETE_ACTION_NAMES):
        raise ValueError('Scene 2 requires the complete P02..P15 action order')
    result, position, heading = [], [0.0, 0.0], 0.0
    for action in actions:
        expected_kind = 'turn' if action['name'].endswith('_after_turn') else 'path'
        if action.get('kind') != expected_kind:
            raise ValueError('Waypoint labels must match their action kind')
        if action['name'] == TERMINAL_ACTION and number(action['heading_rad'], 'heading_rad') != 0:
            raise ValueError('P15_after_turn requires heading_rad: 0.0 (left 180 degrees)')
        clean, position, heading = validate_action(action, position, heading)
        result.append(clean)
    test_until = data.get('test_until')
    if test_until not in tuple(action['name'] for action in actions):
        raise ValueError('Scene 2 requires a declared test_until action')
    return dict(
        schema=1,
        scene=2,
        frame='P01',
        validated=data['validated'],
        test_until=test_until,
        actions=result,
    )


def load_scene(path, scene):
    if type(scene) is not int or scene not in SCENE_FRAMES:
        raise ValueError('Unsupported scene; scene 4 is not implemented')
    data = yaml.load(Path(path).read_text(), Loader=UniqueLoader)
    mapping(data, 'scene file')
    if type(data.get('schema')) is not int or data['schema'] != 1:
        raise ValueError('Expected schema 1')
    if (
        type(data.get('scene')) is not int
        or data['scene'] != scene
        or data.get('frame') != SCENE_FRAMES[scene]
    ):
        raise ValueError(
            f'Scene selection and fixed {SCENE_FRAMES[scene]} frame must match the file'
        )
    allowed = {'schema', 'scene', 'frame', 'source'} | (
        {'parameters'} if scene in (1, 3) else {'validated', 'test_until', 'actions'}
    )
    if data.keys() - allowed:
        raise ValueError('Unknown scene file field')
    return validate_sim(data, scene) if scene in (1, 3) else validate_real(data)


def select_real(route, destination='P15', execute=False):
    label = 'P03_after_turn' if destination == 'P03_turn' else destination
    if label not in COMPLETE_ACTION_NAMES:
        raise ValueError('Unknown destination')
    names = tuple(action['name'] for action in route['actions'])
    if destination == 'P15':
        label = TERMINAL_ACTION
    if label not in names:
        raise ValueError('Full P15 route requires explicit P15_after_turn; update the manifest')
    index = names.index(label)
    permission = route['test_until']
    if permission == 'P15' and TERMINAL_ACTION in names:
        permission = TERMINAL_ACTION
    if execute and not route['validated'] and index > names.index(permission):
        raise ValueError(
            f'Route not validated beyond {route["test_until"]}; requested {destination}'
        )
    return dict(schema=1, scene=2, frame='P01', actions=route['actions'][: index + 1])
