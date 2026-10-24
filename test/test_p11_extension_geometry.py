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
        self.assertEqual(points[2], [0.9307227510453786, -1.8100520547322465])
        self.assertTrue(all(p[1] >= p10[1] for p in points))
        self.assertEqual(points[-1], [0.9221371725514836, -1.0915868730786444])
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

    def test_all_other_geometry_matches_reviewed_691e292_baseline(self):
        # Baseline hashes are offline guards against accidental route-wide translations.
        others = [a for a in self.route['actions'] if a['name'] not in ('P10', 'P11', 'P12')]
        self.assertEqual(
            geometry_hash(others),
            'dc39c65c52ee7249c6e68932127a3a1cd749c1e6e1b68900ba4e6691c0194e25',
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
        self.assertEqual(
            geometry_hash(self.actions['P12']['points'][2:]),
            'aa2284de793e3c235904907b05029bde088d9cee193dbd03085f87d42d94dc72',
        )
        self.assertEqual(self.actions['P12']['heading_rad'], -3 * math.pi / 2)


if __name__ == '__main__':
    unittest.main()
