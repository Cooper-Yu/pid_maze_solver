"""Task7 reverse geometry contracts; pure YAML/math, never a robot or simulator."""

import copy
import math
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate(steps, origin=(0.0, 0.0, 0.0)):
    """Independently integrate turn-before-translation steps in one world frame."""
    x, y, yaw = origin
    points, headings = [(x, y)], []
    for step in steps.values():
        yaw += math.radians(step['turn_deg'])
        c, s = math.cos(yaw), math.sin(yaw)
        x += c * step['forward_m'] - s * step['left_m']
        y += s * step['forward_m'] + c * step['left_m']
        points.append((x, y))
        headings.append(yaw)
    return points, headings


class ReverseGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.forward = yaml.safe_load((ROOT / 'waypoints/waypoints_sim.yaml').read_text())
        cls.reverse = yaml.safe_load((ROOT / 'waypoints/reverse_waypoints_sim.yaml').read_text())
        cls.outbound = cls.forward['parameters']['steps']
        cls.inbound = cls.reverse['parameters']['steps']
        cls.out_points, cls.out_headings = integrate(cls.outbound)
        cls.start_yaw = cls.out_headings[-1] - math.pi
        cls.back_points, cls.back_headings = integrate(
            cls.inbound, (*cls.out_points[-1], cls.start_yaw)
        )

    def assert_point_close(self, actual, expected):
        self.assertLess(math.dist(actual, expected), 1e-12, (actual, expected))

    def test_schema_and_exact_fourteen_reversed_labels(self):
        self.assertEqual((self.reverse['schema'], self.reverse['scene']), (1, 3))
        self.assertEqual(self.reverse['frame'], 'P15')
        self.assertEqual(self.forward['frame'], 'P01')
        self.assertEqual(len(self.outbound), 14)
        expected = [f'P{i:02d}_P{i - 1:02d}' for i in range(15, 1, -1)]
        self.assertEqual(list(self.inbound), expected)
        self.assertEqual(list(self.outbound), [f'P{i:02d}_P{i + 1:02d}' for i in range(1, 15)])

    def test_start_matches_task5_final_clockwise_half_turn(self):
        self.assertIs(self.forward['parameters']['final_clockwise_turn'], True)
        self.assertAlmostEqual(wrap(self.out_headings[-1] - math.pi), 0.0)
        self.assertAlmostEqual(wrap(self.start_yaw), 0.0)
        first = next(iter(self.inbound.values()))
        self.assertEqual(first['turn_deg'], 0.0)
        self.assertAlmostEqual(wrap(self.back_headings[0]), 0.0)
        self.assertIs(self.reverse['parameters']['final_clockwise_turn'], False)

    def test_every_segment_retraces_endpoints_and_interiors(self):
        for index in range(14):
            with self.subTest(reverse_segment=index + 1):
                a, b = self.back_points[index : index + 2]
                expected_a, expected_b = self.out_points[14 - index], self.out_points[13 - index]
                self.assert_point_close(a, expected_a)
                self.assert_point_close(b, expected_b)
                for fraction in (0.25, 0.5, 0.75):
                    self.assert_point_close(
                        tuple(x + fraction * (y - x) for x, y in zip(a, b)),
                        tuple(x + fraction * (y - x) for x, y in zip(expected_a, expected_b)),
                    )
                self.assertAlmostEqual(
                    wrap(self.back_headings[index] - self.out_headings[13 - index] - math.pi),
                    0.0,
                )

    def test_finishes_at_original_p01_facing_pi_without_extra_turn(self):
        self.assert_point_close(self.back_points[-1], (0.0, 0.0))
        self.assertAlmostEqual(wrap(self.back_headings[-1] - math.pi), 0.0)
        self.assertAlmostEqual(abs(self.back_headings[-1] - self.start_yaw), math.pi)

    def test_translations_speed_and_segment_guards_are_preserved(self):
        forward_rows = list(reversed(list(self.outbound.values())))
        for reverse_row, forward_row in zip(self.inbound.values(), forward_rows):
            with self.subTest(forward=forward_row):
                # Facing the opposite way reverses the world vector: do not negate body offsets.
                self.assertEqual(reverse_row['forward_m'], forward_row['forward_m'])
                self.assertEqual(reverse_row['left_m'], forward_row['left_m'])
                original = {k: v for k, v in forward_row.items() if k != 'turn_deg'}
                reversed_parameters = {k: v for k, v in reverse_row.items() if k != 'turn_deg'}
                self.assertEqual(reversed_parameters, original)

    def test_all_nonroute_parameters_preserve_pid_and_safety_configuration(self):
        forward = copy.deepcopy(self.forward['parameters'])
        reverse = copy.deepcopy(self.reverse['parameters'])
        self.assertEqual(reverse.pop('route_direction'), 'reverse')
        self.assertEqual(forward.pop('route_direction', 'forward'), 'forward')
        for data in (forward, reverse):
            data.pop('steps')
            data.pop('final_clockwise_turn')
        # Exact key/value equality also rejects new reverse-only PID/tolerance overrides.
        self.assertEqual(reverse, forward)
        self.assertEqual(reverse['route_mode'], 'motion_steps')
        self.assertIs(reverse['side_centering'], True)

    def test_turns_undo_the_next_outbound_corner_not_the_same_segment(self):
        outgoing = list(self.outbound.values())
        incoming = list(self.inbound.values())
        self.assertEqual(incoming[0]['turn_deg'], 0.0)
        for index in range(1, 14):
            self.assertEqual(incoming[index]['turn_deg'], -outgoing[14 - index]['turn_deg'])
        naive = copy.deepcopy(self.inbound)
        for row, original in zip(naive.values(), reversed(outgoing)):
            row['turn_deg'] = -original['turn_deg']
        points, _ = integrate(naive, (*self.out_points[-1], self.start_yaw))
        self.assertGreater(
            max(
                math.dist(actual, target)
                for actual, target in zip(points, reversed(self.out_points))
            ),
            0.1,
            'Negative control must expose the common reversed-turn indexing error',
        )


if __name__ == '__main__':
    unittest.main()
