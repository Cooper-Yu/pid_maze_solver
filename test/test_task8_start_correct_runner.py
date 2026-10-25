"""No-hardware sequencing, budget and final-acceptance contracts."""

from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_runner import backend_imports

backend_imports()
from action_plan import Pose  # noqa: E402
import task8_start_correct_runner as runtime  # noqa: E402


def proposal(action, x=0.0, yaw=0.0):
    return dict(action=action, forward_m=x, left_m=0.0, angle_rad=yaw)


class Execution(unittest.TestCase):
    def setUp(self):
        self.console = types.SimpleNamespace(
            runner=object(), emit=Mock(), move=Mock(), record=Mock()
        )
        self.cancel = types.SimpleNamespace(check=Mock())
        self.final = Mock(return_value=Pose(1, 2, 0))

    def test_wait_still_requires_original_initializer(self):
        with patch.object(runtime, 'collect', return_value=proposal('wait')):
            result = runtime.correct(self.console, None, self.cancel, self.final)
        self.assertEqual(result.x, 1)
        self.final.assert_called_once_with()
        self.console.move.assert_not_called()

    def test_fresh_batch_after_each_motion(self):
        with patch.object(
            runtime, 'collect', side_effect=[proposal('translate', 0.03), proposal('wait')]
        ) as collect:
            runtime.correct(self.console, None, self.cancel, self.final)
        self.assertEqual(collect.call_count, 2)
        self.assertEqual(self.console.move.call_args.args[0].x, 0.03)
        self.console.record.assert_called_once_with('P15_correction_0')

    def test_final_rejection_propagates(self):
        self.final.side_effect = RuntimeError('Original strict gate rejected')
        with patch.object(runtime, 'collect', return_value=proposal('wait')):
            with self.assertRaisesRegex(RuntimeError, 'Original strict'):
                runtime.correct(self.console, None, self.cancel, self.final)
        self.console.move.assert_not_called()

    def test_step_budget_prevents_additional_motion(self):
        with patch.object(runtime, 'collect', return_value=proposal('translate', 0.03)):
            with self.assertRaisesRegex(RuntimeError, 'step budget'):
                runtime.correct(self.console, None, self.cancel, self.final, max_steps=1)
        self.assertEqual(self.console.move.call_count, 1)
        self.final.assert_not_called()

    def test_total_distance_budget(self):
        with patch.object(runtime, 'collect', return_value=proposal('translate', 0.03)):
            with self.assertRaisesRegex(RuntimeError, 'cumulative'):
                runtime.correct(self.console, None, self.cancel, self.final)
        self.assertEqual(self.console.move.call_count, 5)

    def test_feedback_error_stops_sequence(self):
        self.console.move.side_effect = RuntimeError('4cm guard')
        with patch.object(runtime, 'collect', return_value=proposal('turn', yaw=0.02)):
            with self.assertRaisesRegex(RuntimeError, '4cm'):
                runtime.correct(self.console, None, self.cancel, self.final)
        self.console.record.assert_not_called()
        self.final.assert_not_called()


if __name__ == '__main__':
    unittest.main()
