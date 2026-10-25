"""Owned publisher stop/release and feedback-failure sequencing, no ROS."""

from pathlib import Path
import sys
import time
from types import SimpleNamespace as N
from unittest.mock import Mock, patch
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task8_wall_servo_runner as runtime


def fixture():
    clock = N(value=1.0)
    runner = N()
    runner.pose = lambda: N(x=0.0, y=0.0, yaw=0.0)
    runner.moving_pose = runner.pose
    runner.feedback_pause = N(
        sources={k: N(sample=dict(x=0.0, y=0.0)) for k in ('filtered', 'raw')}
    )
    runner.assert_exclusive = Mock()
    runner.twist = object()
    runner.node = N(
        create_publisher=Mock(return_value=object()),
        get_clock=lambda: N(now=lambda: N(nanoseconds=int(clock.value * 1e9))),
    )
    runner.observer = N(received=time.monotonic())
    runner.send = Mock()

    def pump():
        clock.value += 0.1

    runner.pump = pump
    console = N(runner=runner, emit=Mock(), stop_motion=Mock())
    cancellation = N(check=Mock())

    def sample(*_):
        return dict(
            scan_stamp=int(clock.value * 1e9),
            time=clock.value,
            estimate=dict(x=0.0, y=0.0, yaw=0.0),
        )

    return clock, runner, console, cancellation, sample


def test_ready_hold_stops_and_never_starts_route():
    clock, r, c, cancel, sample = fixture()
    with patch.object(runtime, 'current_sample', side_effect=sample):
        runtime.servo(c, None, cancel, lambda: clock.value)
    assert r.node.create_publisher.call_count == 1
    assert all(call.args[0] == (0.0, 0.0, 0.0) for call in r.send.call_args_list)
    c.stop_motion.assert_called_once_with()


def test_wall_loss_publishes_zeros_then_bounded_exit():
    clock, r, c, cancel, _ = fixture()
    with patch.object(runtime, 'current_sample', side_effect=ValueError('Wall missing')):
        with pytest.raises(RuntimeError, match='30s'):
            runtime.servo(c, None, cancel, lambda: clock.value)
    assert all(call.args[0] == (0.0, 0.0, 0.0) for call in r.send.call_args_list)
    c.stop_motion.assert_called_once_with()


def test_cancel_always_stops_owned_publisher():
    clock, r, c, cancel, sample = fixture()
    cancel.check.side_effect = RuntimeError('cancel')
    with patch.object(runtime, 'current_sample', side_effect=sample):
        with pytest.raises(RuntimeError, match='cancel'):
            runtime.servo(c, None, cancel, lambda: clock.value)
    c.stop_motion.assert_called_once_with()
    r.send.assert_not_called()


def test_raw_odometry_center_drift_always_stops():
    clock, r, c, cancel, sample = fixture()

    def pump():
        clock.value += 0.1
        r.feedback_pause.sources['raw'].sample['x'] = 0.041

    r.pump = pump
    with patch.object(runtime, 'current_sample', side_effect=sample):
        with pytest.raises(RuntimeError, match='4cm'):
            runtime.servo(c, None, cancel, lambda: clock.value)
    c.stop_motion.assert_called_once_with()
    r.send.assert_not_called()


def test_expired_scan_zeroes_then_resumes_only_fresh_samples():
    clock, r, c, cancel, sample = fixture()

    def delayed(*args):
        s = sample(*args)
        if clock.value < 1.7:
            s['scan_stamp'] -= 400_000_000
        return s

    with patch.object(runtime, 'current_sample', side_effect=delayed):
        runtime.servo(c, None, cancel, lambda: clock.value)
    assert any(call.args[0] == 'wall_servo_freshness_wait' for call in c.emit.call_args_list)
    assert all(call.args[0] == (0.0, 0.0, 0.0) for call in r.send.call_args_list)
    c.stop_motion.assert_called_once_with()


def test_zero_wait_time_is_not_charged_as_nonzero_rotation():
    clock, r, c, cancel, sample = fixture()

    def waiting(*args):
        if 1.4 < clock.value < 12:
            raise ValueError('Wall temporarily unavailable')
        value = sample(*args)
        value['estimate']['yaw'] = 0.04 if clock.value < 12.5 else 0.0
        return value

    with patch.object(runtime, 'current_sample', side_effect=waiting):
        runtime.servo(c, None, cancel, lambda: clock.value)
    c.stop_motion.assert_called_once_with()
