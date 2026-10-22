#!/usr/bin/env python3
"""Generate a Task8 candidate from Task6 geometry; no ROS or motion side effects."""

import argparse
import copy
import math
from pathlib import Path

import yaml
from scene_route import COMPLETE_ACTION_NAMES, heading_equal, load_scene, validate_action


def reverse_real(forward):
    """Retrace interpolation in a P15 frame aligned after the outbound left half-turn."""
    if tuple(a['name'] for a in forward['actions']) != COMPLETE_ACTION_NAMES:
        raise ValueError('Task8 requires the complete outbound route and P15 half-turn')
    actions = forward['actions']
    if not heading_equal(actions[-1]['heading_rad'], 0.0):
        raise ValueError('Expected outbound P15 after-turn heading zero')
    endpoint = actions[-2]['points'][-1]
    history, heading = [], 0.0
    for action in actions[:-1]:
        history.append((action, heading))
        heading = action['heading_rad']
    result = []
    for action, previous_heading in reversed(history):
        if action['kind'] == 'turn':
            result.append(
                dict(name=action['name'], kind='turn', heading_rad=previous_heading + math.pi)
            )
        else:
            target = int(action['name'][1:]) - 1
            points = [[p[0] - endpoint[0], p[1] - endpoint[1]] for p in reversed(action['points'])]
            result.append(
                dict(
                    name=f'P{target:02d}',
                    kind='path',
                    heading_rad=action['heading_rad'] + math.pi,
                    points=points,
                )
            )
    result.append(
        dict(name='P01_after_turn', kind='turn', heading_rad=result[-1]['heading_rad'] + math.pi)
    )
    # Reuse continuity, spacing and explicit-heading checks. This does not enable scene4.
    position, heading = [0.0, 0.0], 0.0
    for action in result:
        _, position, heading = validate_action(action, position, heading)
    if not heading_equal(heading, 0.0):
        raise ValueError('Reverse terminal heading must restore outbound P01 orientation')
    return dict(
        schema=1,
        scene=4,
        frame='P15',
        validated=False,
        source='Task8 candidate: reverse Task6 interpolation; P01_after_turn is signed +pi.',
        test_until='P01_after_turn',
        actions=copy.deepcopy(result),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('forward', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    candidate = reverse_real(load_scene(args.forward, 2))
    with args.output.open('x') as stream:
        yaml.safe_dump(candidate, stream, sort_keys=False)
    print(f'Candidate only: {len(candidate["actions"])} actions; no hardware execution enabled')


if __name__ == '__main__':
    main()
