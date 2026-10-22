"""Owned evidence/controller lifetime; stop the official runner before closing evidence."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from task6_preflight import OBSERVATION_SCAN_TIMEOUT, ODOM, SCAN
from task6_feedback_pause import SupervisorPause

EVIDENCE_RATE = (*ODOM, SCAN, '/image_raw/compressed')
EVIDENCE_ONCE = ('/tf_static',)


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    temporary = Path(str(path) + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def spawn(command, log):
    with Path(log).open('x') as output:
        return subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )


def stop_child(child, probe, grace=20):
    if child is None or child.poll() is not None:
        return
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace
    while child.poll() is None and time.monotonic() < deadline:
        probe.pump()
    if child.poll() is None:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired as error:
            # Do not kill a forwarding parent and silently orphan its controller.
            raise RuntimeError('Owned process did not stop; manual stop required') from error


def recorder_health(root, previous=None):
    session = Path((root / 'latest_session.txt').read_text().strip()).resolve()
    if not session.is_relative_to(root.resolve()):
        raise RuntimeError('Recorder session escaped this run directory')
    data = read_json(session / 'health.json')
    if not -1 <= time.time() - data['unix_time'] <= 8:
        raise RuntimeError('Recorder health stale')
    counts = data['message_counts']
    if any(counts.get(t, 0) <= 0 for t in (*EVIDENCE_RATE, *EVIDENCE_ONCE)):
        raise RuntimeError('Recorder missing required odometry/scan/image/static TF')
    if previous is not None and any(counts[t] <= previous[t] for t in EVIDENCE_RATE):
        raise RuntimeError('Recorder required feedback counts did not grow')
    return session, data


class StartupScanWait:
    """Bound one stopped scan outage; fresh timestamps, not loop ticks, recover it."""

    def __init__(self):
        self.started = None
        self.last_stamp = None
        self.fresh_count = 0

    def ready(self, snapshot, now):
        stamp = snapshot['samples'][SCAN]['stamp']
        if not snapshot['scan_fresh'] and self.started is None:
            self.started, self.last_stamp = now, stamp
        if self.started is None:
            return True
        if now - self.started >= 2.0:
            raise RuntimeError('Evidence startup scan wait exceeded 2 seconds')
        if not snapshot['scan_fresh']:
            self.last_stamp, self.fresh_count = stamp, 0
        elif stamp > self.last_stamp:
            self.last_stamp = stamp
            self.fresh_count += 1
        if self.fresh_count < 3:
            return False
        self.started, self.last_stamp, self.fresh_count = None, None, 0
        return True


class Operation:
    def __init__(self, directory, config, probe, identities):
        self.directory, self.config, self.probe, self.identities = (
            directory,
            config,
            probe,
            identities,
        )
        self.recorder = self.watch = self.runner = None
        self.recorder_session = None
        self.cancelled = False
        self.started = time.monotonic()
        self.last_health = None
        self.last_growth = None
        self.phase = 'created'

    def cancel(self, *_):
        self.cancelled = True

    def check_limits(self):
        if self.cancelled:
            raise RuntimeError('Cancelled')
        if time.monotonic() - self.started >= self.config['max_seconds']:
            raise RuntimeError('Maximum run duration reached')
        if time.time() >= self.config['deadline_unix'] - 90:
            raise RuntimeError('Booking deadline cleanup reserve reached')

    def start_evidence(self):
        self.phase = 'evidence_startup'
        support = Path(self.config['support'])
        seconds = str(min(1800, self.config['max_seconds'] + 90))
        self.recorder = spawn(
            [
                sys.executable,
                str(support / 'evidence_recorder.py'),
                '--root',
                str(self.directory / 'evidence'),
                '--seconds',
                seconds,
                '--image-hz',
                '2',
                '--max-mib',
                '180',
            ],
            self.directory / 'recorder.log',
        )
        self.watch = spawn(
            [
                sys.executable,
                str(support / 'odom_reset_watch.py'),
                '--output',
                str(self.directory / 'odom_watch.jsonl'),
                '--seconds',
                seconds,
                '--base-topic',
                ODOM[1],
            ],
            self.directory / 'watch.log',
        )
        deadline, first = time.monotonic() + 18, None
        scan_wait = StartupScanWait()
        while time.monotonic() < deadline:
            self.probe.pump()
            self.check_limits()
            observed = self.probe.snapshot(
                stopped=True, expected=self.identities, stopped_scan_wait=True
            )
            scan_ready = scan_wait.ready(observed, time.monotonic())
            self.require_evidence_alive()
            try:
                session, health = recorder_health(self.directory / 'evidence')
                if first is None:
                    first = health
                    continue
                if health['unix_time'] == first['unix_time']:
                    continue
                recorder_health(self.directory / 'evidence', first['message_counts'])
            except (FileNotFoundError, json.JSONDecodeError):
                continue
            except RuntimeError:
                if first is not None:
                    raise
                continue
            if scan_ready:
                self.probe.snapshot(stopped=True, expected=self.identities)
                self.recorder_session, self.last_health = session, health
                self.last_growth = time.monotonic()
                return
        raise RuntimeError('Recorder did not prove required topic growth before motion')

    def require_evidence_alive(self):
        if any(p is not None and p.poll() is not None for p in (self.recorder, self.watch)):
            raise RuntimeError('Evidence process ended; stopping route')

    def check_recording(self):
        self.require_evidence_alive()
        session, health = recorder_health(self.directory / 'evidence')
        if health['unix_time'] != self.last_health['unix_time']:
            previous = (
                None
                if getattr(self, 'feedback_waiting', False)
                else self.last_health['message_counts']
            )
            recorder_health(self.directory / 'evidence', previous)
            self.last_health, self.last_growth = health, time.monotonic()
        if time.monotonic() - self.last_growth > 8:
            raise RuntimeError('Recorder growth timeout')
        self.recorder_session = session

    def execute(self):
        self.start_evidence()
        self.check_limits()
        # No session import: official scene2 always verifies a fresh P01 itself.
        command = [
            self.config['entry'],
            '2',
            '--destination',
            self.config['destination'],
            '--waypoints',
            self.config['waypoints'],
            '--output',
            str(self.directory / 'points.json'),
            '--speed',
            str(self.config.get('speed', 0.04)),
            '--turn-speed',
            str(self.config.get('turn_speed', 0.10)),
        ]
        write_json(self.directory / 'motion_command.json', command)
        self.phase = 'official_runner'
        self.check_limits()
        self.probe.snapshot(stopped=True, expected=self.identities)
        self.runner = spawn(command, self.directory / 'motion.log')
        feedback_pause = SupervisorPause(self.directory)
        while self.runner.poll() is None:
            self.probe.pump()
            self.check_limits()
            feedback_pause.observe(
                self.probe,
                self.identities,
                scan_timeout=(
                    OBSERVATION_SCAN_TIMEOUT
                    if self.config.get('laser_policy') == 'observation_only'
                    else 0.5
                ),
            )
            self.feedback_waiting = feedback_pause.acknowledged
            self.check_recording()
        if self.runner.returncode != 0:
            raise RuntimeError(f'Official scene2 exited {self.runner.returncode}')
        rows = [
            json.loads(line) for line in (self.directory / 'points.jsonl').read_text().splitlines()
        ]
        if (
            not rows
            or rows[-1].get('event') != 'route_completed'
            or rows[-1].get('destination') != self.config['destination']
        ):
            raise RuntimeError('No matching route_completed evidence')
        if self.config['destination'] == 'P15' and (
            rows[-1].get('final_action') != 'P15_after_turn'
            or not any(
                row.get('event') == 'route_action_completed' and row.get('name') == 'P15_after_turn'
                for row in rows[:-1]
            )
        ):
            raise RuntimeError('No completed P15_after_turn terminal evidence')

    def verify_final_stop(self):
        zero = self.probe.wait_zero(self.identities) if self.runner is not None else True
        return dict(zero_and_released=zero, stopped_and_released=zero)

    def cleanup(self):
        failures = []
        try:
            stop_child(self.runner, self.probe)
            stop = self.verify_final_stop()
            if not stop['stopped_and_released']:
                failures.append('Final zero + publisher release + stopped feedback not verified')
        except Exception as error:
            stop = dict(zero_and_released=False, stopped_and_released=False)
            failures.append(str(error))
        for child in (self.recorder, self.watch):
            try:
                stop_child(child, self.probe)
            except Exception as error:
                failures.append(str(error))
        evidence = None
        if self.recorder is not None:
            try:
                session = Path((self.directory / 'evidence/latest_session.txt').read_text().strip())
                evidence = read_json(session / 'result.json')
                if not evidence.get('closed_metadata') or evidence.get('exception'):
                    failures.append('Recorder metadata close failed')
                if not (session / 'bag/metadata.yaml').is_file():
                    failures.append('Recorder metadata file missing')
            except (OSError, ValueError) as error:
                failures.append('Recorder close not verified: ' + str(error))
        return dict(**stop, evidence=evidence, failures=failures)

    def capture_failure(self, error):
        """Best-effort pre-cleanup evidence; diagnostic failure must never skip stopping."""
        evidence = dict(
            error=error,
            unix_time=time.time(),
            phase=self.phase,
            runner_returncode=self.runner.poll() if self.runner is not None else None,
        )
        try:
            evidence['probe'] = self.probe.diagnostics()
        except Exception as diagnostic_error:
            evidence['probe_error'] = str(diagnostic_error)
        return evidence

    def run(self):
        handlers = {sig: signal.signal(sig, self.cancel) for sig in (signal.SIGINT, signal.SIGTERM)}
        error = diagnostic_error = failure_snapshot = None
        try:
            self.execute()
        except Exception as failure:
            error = str(failure)
            try:
                failure_snapshot = self.capture_failure(error)
            except Exception as capture_error:
                diagnostic_error = str(capture_error)
        finally:
            cleanup = self.cleanup()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
        # Capture precedes stop, but disk I/O must not delay sending the stop signal.
        if failure_snapshot is not None:
            try:
                write_json(self.directory / 'failure_snapshot.json', failure_snapshot)
            except Exception as capture_error:
                diagnostic_error = str(capture_error)
        result = dict(
            success=error is None and not cleanup['failures'],
            error=error,
            failure_snapshot_error=diagnostic_error,
            cleanup=cleanup,
            elapsed_s=time.monotonic() - self.started,
            destination=self.config['destination'],
            task6_acceptance=False,
            laser_policy='observation_only',
        )
        write_json(self.directory / 'result.json', result)
        return 0 if result['success'] else 2
