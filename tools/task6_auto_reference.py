"""Own scene2 evidence and publish a reference only after the route and bag close."""

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from task8_reference import default_registry, register

REQUIRED = (
    '/odometry/filtered',
    '/rosbot_xl_base_controller/odom',
    '/scan_filtered',
    '/image_raw/compressed',
    '/tf_static',
)


@contextmanager
def owner_lock():
    import fcntl

    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdecimal() or not 0 <= int(domain) <= 232:
        raise ValueError('Invalid ROS_DOMAIN_ID')
    directory = Path.home() / '.cache/pid_maze_solver'
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / f'forward_reference_{int(domain)}.lock').open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another automatic forward session owns this ROS domain') from None
        yield


class Recording:
    """One read-only recorder subprocess, never a velocity publisher."""

    def __init__(self, directory, seconds):
        self.root = Path(directory) / 'evidence'
        self.seconds = seconds
        self.child = self.session = self.last_health = None
        self.growth = None

    def start(self, cancelled):
        recorder = Path(__file__).parent / 'reference_evidence/evidence_recorder.py'
        with (self.root.parent / 'recorder.log').open('x') as log:
            self.child = subprocess.Popen(
                [
                    sys.executable,
                    str(recorder),
                    '--root',
                    str(self.root),
                    '--seconds',
                    str(math.ceil(self.seconds + 60)),
                    '--image-hz',
                    '2',
                    '--max-mib',
                    '256',
                ],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        deadline = time.monotonic() + 18
        while time.monotonic() < deadline:
            if cancelled():
                raise InterruptedError('Cancelled before motion')
            try:
                self.check()
                return
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            time.sleep(0.1)
        raise RuntimeError('Reference recorder did not receive required topics before motion')

    def check(self):
        if self.child.poll() is not None:
            raise RuntimeError('Reference recorder exited; route must stop')
        session = Path((self.root / 'latest_session.txt').read_text().strip()).resolve()
        if not session.is_relative_to(self.root.resolve()):
            raise RuntimeError('Reference recorder escaped its run directory')
        self.session = session
        health = json.loads((session / 'health.json').read_text())
        if not -1 <= time.time() - health['unix_time'] <= 8:
            raise RuntimeError('Reference recorder health stale')
        counts = health['message_counts']
        if not all(counts.get(topic, 0) > 0 for topic in REQUIRED):
            if self.last_health is None:
                raise FileNotFoundError('Waiting for reference topics')
            raise RuntimeError('Reference recorder lost required topics')
        if self.last_health is None:
            self.growth = time.monotonic()
        elif health['unix_time'] != self.last_health['unix_time']:
            dynamic = REQUIRED[:-1]
            if not all(counts[t] > self.last_health['message_counts'][t] for t in dynamic):
                raise RuntimeError('Reference recorder topic growth stopped')
            self.growth = time.monotonic()
        if time.monotonic() - self.growth > 8:
            raise RuntimeError('Reference recorder growth timeout')
        self.last_health = health

    def signal(self, signum):
        try:
            os.killpg(self.child.pid, signum)
        except ProcessLookupError:
            pass

    def close(self):
        """Stop only our recorder; a closure failure prevents registration."""
        if self.child is None:
            return
        if self.child.poll() is None:
            self.signal(signal.SIGINT)
            try:
                self.child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.signal(signal.SIGTERM)
                try:
                    self.child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.signal(signal.SIGKILL)
                    self.child.wait(timeout=3)
                    raise RuntimeError('Reference recorder could not close cleanly') from None
        if self.child.returncode != 0 or self.session is None:
            raise RuntimeError('Reference recorder failed; previous reference retained')
        result = json.loads((self.session / 'result.json').read_text())
        if result.get('closed_metadata') is not True or result.get('exception') is not None:
            raise RuntimeError('Reference evidence did not close cleanly')


def run_recorded_forward(command, forward_route, run_owned, max_seconds=600):
    """Return success only when full-route evidence is registered, without another motion."""
    with owner_lock():
        parent = default_registry().parent / 'runs'
        parent.mkdir(parents=True, exist_ok=True)
        directory = Path(tempfile.mkdtemp(prefix='forward_', dir=parent))
        output = directory / 'route.json'
        recording = Recording(directory, max_seconds)
        cancelled = []
        previous = {}
        started = time.monotonic()

        def cancel(signum, _frame):
            cancelled.append(signum)

        def monitor():
            if time.monotonic() - started >= max_seconds:
                raise RuntimeError('Automatic forward duration limit reached')
            recording.check()

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, cancel)
        try:
            print('FORWARD_REFERENCE_RUN:', directory, flush=True)
            try:
                recording.start(lambda: bool(cancelled))
                if cancelled:
                    raise InterruptedError('Cancelled before motion')
                result = run_owned([*command, '--output', str(output)], monitor=monitor)
            finally:
                recording.close()
            if result != 0 or cancelled:
                return 128 + cancelled[0] if cancelled else result
            target = register(output, recording.session / 'bag/bag_0.db3', forward_route)
            print('P15_REFERENCE_REGISTERED_NO_MOTION:', target, flush=True)
            return 0
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)
