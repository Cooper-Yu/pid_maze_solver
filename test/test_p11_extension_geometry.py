"""Offline scope and sampling checks; these do not prove physical wall clearance."""

import hashlib
import json
import math
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from scene_route import validate_real  # noqa: E402


def geometry_hash(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
    ).hexdigest()


class P11ExtensionGeometry(unittest.TestCase):
    def setUp(self):
        self.route = yaml.safe_load((ROOT / 'waypoints/waypoints_real.yaml').read_text())
        self.actions = {action['name']: action for action in self.route['actions']}

    def test_extension_is_five_centimetres_forward_after_p10_turn(self):
        p11 = self.actions['P11']
        before, after = p11['points'][-2:]
        heading = self.actions['P10_after_turn']['heading_rad']
        self.assertEqual(p11['heading_rad'], heading)
        self.assertEqual(heading, -math.pi)
        dx, dy = after[0] - before[0], after[1] - before[1]
        self.assertAlmostEqual(dx * math.cos(heading) + dy * math.sin(heading), 0.05)
        self.assertAlmostEqual(-dx * math.sin(heading) + dy * math.cos(heading), 0.0)
        self.assertEqual(after, [0.8818674948445645, -1.8579827169427832])

    def test_p11_stays_straight_and_p12_rejoins_without_retreat_to_old_wall(self):
        p10 = self.actions['P10']['points'][-1]
        self.assertTrue(all(p[1] == p10[1] for p in self.actions['P11']['points']))
        points = self.actions['P12']['points']
        self.assertEqual(points[0], self.actions['P11']['points'][-1])
        self.assertTrue(all(p[0] == self.actions['P11']['points'][-1][0] for p in points))
        self.assertTrue(all(p[1] >= p10[1] for p in points))
        self.assertEqual(points[-1], [0.8818674948445645, -1.0915868730786444])
        self.assertEqual(self.actions['P13']['points'][0], points[-1])
        for first, second in zip(points, points[1:]):
            self.assertTrue(0 < math.dist(first, second) <= 0.050000001)
        validate_real(self.route)
        self.assertFalse(self.route['validated'])

    def test_p10_stages_body_right_only_after_first_twelve_centimetres(self):
        points = self.actions['P10']['points']
        start = points[0]
        end = points[-1]
        self.assertAlmostEqual(end[0], 1.335841813224953 - 0.08)
        self.assertAlmostEqual(end[1], -1.8179827169427831 - 0.04)
        self.assertTrue(all(a[0] >= b[0] and a[1] > b[1] for a, b in zip(points, points[1:])))
        length = start[1] - end[1]
        for x, y in points:
            forward = start[1] - y
            baseline = start[0] + (1.335841813224953 - start[0]) * forward / length
            right = baseline - x
            self.assertGreaterEqual(right, -1e-12)
            self.assertLessEqual(right, 0.0800000001)
            if forward <= 0.120000001:
                self.assertAlmostEqual(right, 0.0)

    def test_p13_right_then_forward_and_p14_line_continuity(self):
        before, right, forward = self.actions['P13']['points'][-3:]
        self.assertEqual(right[0], before[0])
        self.assertAlmostEqual(right[1] - before[1], 0.02)
        self.assertAlmostEqual(forward[0] - right[0], -0.03)
        self.assertEqual(forward[1], right[1])
        p14 = self.actions['P14']['points']
        self.assertEqual(p14[0], forward)
        self.assertTrue(all(p[0] <= forward[0] + 1e-12 for p in p14))
        self.assertTrue(all(b[1] > a[1] for a, b in zip(p14, p14[1:])))
        self.assertEqual(self.actions['P15']['points'][0], p14[-1])
        self.assertEqual(
            self.actions['P15']['points'][-1], [0.02899960878708014, -0.5796575672149936]
        )
        validate_real(self.route)
        self.assertFalse(self.route['validated'])

    def test_p14_and_p15_move_left_two_cm_in_their_respective_headings(self):
        p14, p15 = self.actions['P14'], self.actions['P15']
        self.assertEqual(p14['points'][0], self.actions['P13']['points'][-1])
        self.assertAlmostEqual(p14['points'][-1][0], 0.39248777781822897 - 0.02)
        self.assertAlmostEqual(p14['points'][-1][1], -0.5732626964522145)
        self.assertEqual(p15['points'][0], p14['points'][-1])
        self.assertTrue(all(b[0] < a[0] for a, b in zip(p15['points'], p15['points'][1:])))
        self.assertAlmostEqual(p15['points'][-1][0], 0.02899960878708014)
        self.assertAlmostEqual(p15['points'][-1][1], -0.5596575672149936 - 0.02)
        self.assertEqual(self.actions['P15_after_turn']['heading_rad'], 0.0)
        validate_real(self.route)

    def test_all_other_geometry_matches_reviewed_691e292_baseline(self):
        # Baseline hashes are offline guards against accidental route-wide translations.
        others = [
            a
            for a in self.route['actions']
            if a['name'] not in ('P10', 'P11', 'P12', 'P13', 'P14', 'P15')
        ]
        self.assertEqual(
            geometry_hash(others),
            '0ecb2fbf179f5992120356842d7bb2445788a755e7c38cdc7ace5fbf91c14e2a',
        )
        self.assertEqual(
            self.actions['P10']['points'][-1], [1.255841813224953, -1.8579827169427832]
        )
        self.assertEqual(self.actions['P11']['points'][0], self.actions['P10']['points'][-1])
        self.assertEqual(self.actions['P10']['points'][0], self.actions['P09']['points'][-1])
        for name in ('P10', 'P11'):
            points = self.actions[name]['points']
            self.assertTrue(
                all(0 < math.dist(x, y) <= 0.050000001 for x, y in zip(points, points[1:]))
            )
        self.assertEqual(self.actions['P12']['heading_rad'], -3 * math.pi / 2)


if __name__ == '__main__':
    unittest.main()
