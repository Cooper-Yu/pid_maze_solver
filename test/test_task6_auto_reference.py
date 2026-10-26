"""Automatic registration lifecycle, including failures and actual owned process stop."""

import json
import os
from pathlib import Path
import signal
import sys
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import maze_entry as entry  # noqa: E402
import task6_auto_reference as auto  # noqa: E402
import task8_reference as ref  # noqa: E402
from scene_route import load_scene  # noqa: E402

ROUTE = ROOT / 'waypoints/waypoints_real.yaml'


@pytest.fixture
def session(tmp_path, monkeypatch):
    registry = tmp_path / 'p15_reference.json'
    registry.write_text('previous reference must survive failed runs')
    monkeypatch.setattr(auto, 'default_registry', lambda: registry)
    monkeypatch.setattr(ref, 'default_registry', lambda: registry)
    monkeypatch.setenv('HOME', str(tmp_path))
    log = []

    class Recorder:
        def __init__(self, directory, seconds):
            self.session = Path(directory) / 'evidence/session'
            self.session.mkdir(parents=True)
            self.seconds = seconds

        def start(self, cancelled):
            assert not cancelled()
            log.append('recording_ready')

        def check(self):
            log.append('health_checked')

        def close(self):
            log.append('closed')
            bag = self.session / 'bag'
            bag.mkdir()
            (bag / 'bag_0.db3').write_bytes(b'synthetic, not a ROS acceptance test')
            (bag / 'metadata.yaml').write_text('synthetic: true')
            (self.session / 'result.json').write_text(
                json.dumps(dict(closed_metadata=True, exception=None))
            )

    monkeypatch.setattr(auto, 'Recording', Recorder)
    return registry, log, Recorder


def forward_backend(command, monitor):
    monitor()
    output = Path(command[command.index('--output') + 1])
    output.write_text(json.dumps(dict(frame='odom')))
    actions = load_scene(ROUTE, 2)['actions']
    rows = [dict(event='route_started', scene=2, actions=actions)]
    rows.extend(dict(event='route_action_completed', name=a['name']) for a in actions)
    rows.append(
        dict(event='route_completed', destination='P15', final_action='P15_after_turn', exit_code=0)
    )
    output.with_suffix('.jsonl').write_text('\n'.join(json.dumps(x) for x in rows))
    return 0


def test_forward_closes_then_registers_and_reverse_resolves(session, monkeypatch):
    registry, log, _ = session
    real_register = auto.register

    def register_after_close(*args):
        assert log[-1] == 'closed'
        return real_register(*args)

    monkeypatch.setattr(auto, 'register', register_after_close)
    assert auto.run_recorded_forward(['backend'], ROUTE, forward_backend) == 0
    book, bag = ref.resolve(ROUTE, registry)
    assert book.name == 'route.json' and bag.name == 'bag_0.db3'
    assert log[:2] == ['recording_ready', 'health_checked']


@pytest.mark.parametrize('exit_code', [2, 130, 143])
def test_route_failure_or_cancel_never_overwrites_reference(session, exit_code):
    registry, log, _ = session
    before = registry.read_bytes()
    assert auto.run_recorded_forward(['backend'], ROUTE, lambda *a, **k: exit_code) == exit_code
    assert registry.read_bytes() == before and log[-1] == 'closed'


def test_exit_zero_without_route_evidence_is_not_success(session):
    registry, log, _ = session
    before = registry.read_bytes()
    with pytest.raises(FileNotFoundError):
        auto.run_recorded_forward(['backend'], ROUTE, lambda *a, **k: 0)
    assert registry.read_bytes() == before and log[-1] == 'closed'


def test_closure_failure_preserves_reference(session, monkeypatch):
    registry, _, recorder = session
    before = registry.read_bytes()
    monkeypatch.setattr(recorder, 'close', Mock(side_effect=RuntimeError('closure failed')))
    with pytest.raises(RuntimeError, match='closure failed'):
        auto.run_recorded_forward(['backend'], ROUTE, forward_backend)
    assert registry.read_bytes() == before


def test_startup_cancellation_closes_recorder_without_motion(session, monkeypatch):
    registry, log, recorder = session
    before = registry.read_bytes()
    monkeypatch.setattr(
        recorder,
        'start',
        lambda self, cancelled: signal.getsignal(signal.SIGINT)(signal.SIGINT, None),
    )
    backend = Mock()
    with pytest.raises(InterruptedError):
        auto.run_recorded_forward(['backend'], ROUTE, backend)
    backend.assert_not_called()
    assert registry.read_bytes() == before and log[-1] == 'closed'


def test_recorder_start_failure_closes_without_launch(session, monkeypatch):
    registry, log, recorder = session
    monkeypatch.setattr(recorder, 'start', Mock(side_effect=RuntimeError('no feedback')))
    backend = Mock()
    with pytest.raises(RuntimeError, match='no feedback'):
        auto.run_recorded_forward(['backend'], ROUTE, backend)
    backend.assert_not_called()
    assert registry.read_text().startswith('previous') and log[-1] == 'closed'


def test_monitor_failure_stops_real_owned_process(tmp_path):
    pidfile = tmp_path / 'owned.pid'
    code = (
        'import os,pathlib,time; pathlib.Path('
        + repr(str(pidfile))
        + ').write_text(str(os.getpid())); time.sleep(30)'
    )
    original = signal.getsignal(signal.SIGINT)

    def monitor():
        if pidfile.exists():
            raise RuntimeError('recorder lost')

    with pytest.raises(RuntimeError, match='recorder lost'):
        entry.run_owned([sys.executable, '-c', code], monitor=monitor)
    assert signal.getsignal(signal.SIGINT) is original
    with pytest.raises(ProcessLookupError):
        os.kill(int(pidfile.read_text()), 0)


@pytest.mark.parametrize(
    'arguments,automatic',
    [
        (['2'], True),
        (['2', '--output', '/tmp/explicit.json'], False),
        (['2', '--destination', 'P13'], False),
        (['2', '--supervised'], False),
    ],
)
def test_entry_selects_recording_only_for_complete_automatic_forward(
    monkeypatch, arguments, automatic
):
    owned, recorded = Mock(return_value=0), Mock(return_value=0)
    monkeypatch.setattr(entry, 'executable', lambda *a: '/fake')
    monkeypatch.setattr(entry, 'run_owned', owned)
    monkeypatch.setattr(entry, 'run_recorded_forward', recorded)
    assert entry.main(arguments) == 0
    assert recorded.call_count == int(automatic)
    assert owned.call_count == int(not automatic)


def test_dry_run_never_starts_recorder(monkeypatch):
    recorded = Mock()
    monkeypatch.setattr(entry, 'run_recorded_forward', recorded)
    assert entry.main(['2', '--dry-run']) == 0
    recorded.assert_not_called()


@pytest.mark.parametrize('fault', ['stale', 'exited', 'escape', 'growth'])
def test_recording_health_faults_reject(tmp_path, fault):
    import time

    recording = auto.Recording(tmp_path, 30)
    recording.root.mkdir()
    recording.session = recording.root / 'session'
    recording.session.mkdir()
    (recording.root / 'latest_session.txt').write_text(str(recording.session))
    recording.child = Mock()
    recording.child.poll.return_value = 2 if fault == 'exited' else None
    health = dict(
        unix_time=time.time() - (9 if fault == 'stale' else 0),
        message_counts={t: 1 for t in auto.REQUIRED},
    )
    (recording.session / 'health.json').write_text(json.dumps(health))
    if fault == 'escape':
        (recording.root / 'latest_session.txt').write_text(str(tmp_path))
    if fault == 'growth':
        recording.last_health = dict(
            unix_time=health['unix_time'] - 5, message_counts=health['message_counts']
        )
        recording.growth = time.monotonic()
    with pytest.raises(RuntimeError):
        recording.check()
