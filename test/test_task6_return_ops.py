"""No automatic advance, replay admission and original-epoch boundaries for return wrapper."""

from io import StringIO
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task6_ops_runtime import write_json
from task6_preflight import ODOM
from task6_return_ops import ReturnOperation, audit_rows, load_run, send, start


def operation(directory):
    source = SimpleNamespace(actions=['path', 'turn'], session_id='source')
    op = ReturnOperation(
        directory,
        dict(max_seconds=60, deadline_unix=time.time() + 300),
        Mock(),
        {},
        source,
        directory / 'points.json',
        directory,
    )
    op.runner = SimpleNamespace(stdin=StringIO())
    return op


class ReturnOpsTest(unittest.TestCase):
    def test_attached_waits_and_each_next_requires_separate_request(self):
        with tempfile.TemporaryDirectory() as folder:
            op = operation(Path(folder))
            rows = [dict(event='return_session_started')]
            op.save_state(rows)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            first = dict(id='one', command='next', wall_s=time.time())
            op.request(first, rows)
            self.assertTrue(op.busy)
            op.request(dict(id='two', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\n')
            self.assertFalse(op.state['last_request']['accepted'])
            rows.append(dict(event='return_action_completed', cursor=1, name='P03'))
            op.save_state(rows)
            self.assertFalse(op.busy)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\n')
            op.request(first, rows)
            self.assertFalse(op.state['last_request']['accepted'])
            op.request(dict(id='three', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\nnext\n')

    def test_stale_unknown_and_pre_attach_next_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            op = operation(Path(folder))
            for request in (
                dict(id='old', command='next', wall_s=time.time() - 10),
                dict(id='bad', command='reset', wall_s=time.time()),
                dict(id='early', command='next', wall_s=time.time()),
            ):
                op.request(request, [])
                self.assertFalse(op.state['last_request']['accepted'])
            self.assertEqual(op.runner.stdin.getvalue(), '')

    def test_fifo_mode_single_request_and_closed_refusal(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'control.fifo'
            os.mkfifo(path, 0o600)
            fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
            try:
                write_json(
                    root / 'control.json',
                    dict(ready=True, busy=False, closed=False, wall_s=time.time()),
                )
                send(root, 'next')
                request = json.loads(os.read(fd, 4096))
                self.assertEqual(request['command'], 'next')
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                write_json(
                    root / 'control.json',
                    dict(ready=True, busy=False, closed=True, wall_s=time.time()),
                )
                with self.assertRaisesRegex(RuntimeError, 'closed'):
                    send(root, 'next')
            finally:
                os.close(fd)

    def test_partial_audit_write_is_not_parsed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'audit.jsonl'
            path.write_text('{"event":"ok"}\n{"event":')
            self.assertEqual(audit_rows(path), [dict(event='ok')])

    def test_unsuccessful_or_unclosed_outward_refused_before_import(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write_json(root / 'config.json', {})
            for result in (
                dict(success=False),
                dict(
                    success=True,
                    cleanup=dict(zero_and_released=True, evidence=dict(closed_metadata=False)),
                ),
            ):
                write_json(root / 'result.json', result)
                with self.assertRaisesRegex(ValueError, 'did not complete'):
                    load_run(root)

    def test_new_epoch_refused_without_creating_return_dir(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            gids = {t: ['old'] for t in ODOM}
            args = SimpleNamespace(
                source_run_dir=root,
                return_dir=root / 'return',
                deadline=time.time() + 300,
                max_seconds=60,
            )
            fake = Mock()
            fake.ready.return_value = dict(gids={t: ['new'] for t in ODOM})
            with (
                patch('task6_return_ops.load_run', return_value=({}, Mock(), gids, root)),
                patch('task6_return_ops.Probe', return_value=fake),
            ):
                with self.assertRaisesRegex(RuntimeError, 'GID differs'):
                    start(args)
            self.assertFalse(args.return_dir.exists())
            fake.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
