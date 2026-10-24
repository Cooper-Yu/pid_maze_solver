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
        self.assertEqual(after, [0.8818674948445645, -1.9058474122860602])

    def test_p12_rejoins_old_second_sample_without_breaking_five_cm_spacing(self):
        points = self.actions['P12']['points']
        self.assertEqual(points[0], self.actions['P11']['points'][-1])
        self.assertEqual(points[2], [0.9312951229449715, -1.8579497335091535])
        for axis in (0, 1):
            self.assertAlmostEqual(points[1][axis], (points[0][axis] + points[2][axis]) / 2)
        self.assertGreater(math.dist(points[0], points[2]), 0.05)
        for first, second in zip(points, points[1:]):
            self.assertTrue(0 < math.dist(first, second) <= 0.050000001)
        validate_real(self.route)
        self.assertFalse(self.route['validated'])

    def test_all_other_geometry_matches_reviewed_691e292_baseline(self):
        # Baseline hashes are offline guards against accidental route-wide translations.
        others = [a for a in self.route['actions'] if a['name'] not in ('P10', 'P11', 'P12')]
        self.assertEqual(
            geometry_hash(others),
            'dc39c65c52ee7249c6e68932127a3a1cd749c1e6e1b68900ba4e6691c0194e25',
        )
        self.assertEqual(
            self.actions['P10']['points'][-1], [1.335841813224953, -1.8179827169427831]
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
            '8640ad523f383d48fd0f13afda6f94c3b2fe505146417db6b361d4ef62c9096d',
        )
        self.assertEqual(self.actions['P12']['heading_rad'], -3 * math.pi / 2)


if __name__ == '__main__':
    unittest.main()
