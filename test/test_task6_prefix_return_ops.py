"""Prefix wrapper admission and manual lifecycle tests; no ROS nodes or motion."""

from contextlib import nullcontext
from dataclasses import asdict, dataclass
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task6_prefix_return_ops as prefix
from task6_ops import digest
from task6_ops_runtime import write_json
from task6_preflight import ODOM, SCAN
from task6_return_ops import load_run


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def fixture(root):
    for name in (
        'route_prefix_return_runner',
        'route_prefix_return_plan.py',
        'route_prefix_return_runtime.py',
        'route_return_runner',
        'core',
    ):
        (root / name).write_text(name)
    (root / 'route_return_runner.py').write_bytes((root / 'route_return_runner').read_bytes())
    package = dict(library=str(root), installed_sha256={'core': digest(root / 'core')})
    config = dict(packages={name: package for name in ('distance_controller', 'turn_controller')})
    write_json(root / 'config.json', config)
    write_json(root / 'result.json', dict(success=False))
    source = SimpleNamespace(
        session_id='session', source_hash='hash', actions=['P02', 'P03'], origin=Pose()
    )
    metadata = dict(
        gids={topic: [topic] for topic in ODOM}, source_hash='hash', prefix_names=['P02', 'P03']
    )
    module = SimpleNamespace(
        __file__=str(root / 'route_prefix_return_plan.py'),
        load_prefix_run=Mock(return_value=(source, metadata)),
    )
    return config, source, metadata, module


class PrefixTests(unittest.TestCase):
    def test_separate_validator_does_not_promote_failed_original(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            _config, source, _meta, module = fixture(root)
            with patch.object(prefix.importlib, 'import_module', return_value=module):
                result = prefix.load_prefix(root)
            self.assertIs(result[1], source)
            module.load_prefix_run.assert_called_once_with(root)
            self.assertFalse(json.loads((root / 'result.json').read_text())['success'])
            with self.assertRaisesRegex(ValueError, 'did not complete'):
                load_run(root)

    def test_backend_rejection_cannot_fall_back_to_success_loader(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            *_rest, module = fixture(root)
            module.load_prefix_run.side_effect = ValueError('partial turn refused')
            with patch.object(prefix.importlib, 'import_module', return_value=module):
                with self.assertRaisesRegex(ValueError, 'partial turn'):
                    prefix.load_prefix(root)

    def test_artifact_change_rejected_before_backend(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            fixture(root)
            (root / 'core').write_text('changed')
            with patch.object(prefix.importlib, 'import_module') as imported:
                with self.assertRaisesRegex(ValueError, 'artifact changed'):
                    prefix.load_prefix(root)
                imported.assert_not_called()

    def test_wrong_module_claim_and_metadata_refused(self):
        for failure in ('module', 'claim', 'hash', 'gids', 'shared'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                _config, _source, meta, module = fixture(root)
                if failure == 'module':
                    module.__file__ = str(root / 'elsewhere.py')
                elif failure == 'claim':
                    (root / 'points.return_claim.json').write_text('{}')
                elif failure == 'hash':
                    meta['source_hash'] = 'other'
                elif failure == 'shared':
                    (root / 'route_return_runner.py').write_text('different')
                else:
                    meta['gids'][ODOM[1]] = ['one', 'two']
                with patch.object(prefix.importlib, 'import_module', return_value=module):
                    with self.assertRaises(ValueError):
                        prefix.load_prefix(root)

    def test_both_live_epochs_must_match_before_new_output(self):
        for changed in ODOM:
            with self.subTest(topic=changed), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                config, source, meta, _module = fixture(root)
                args = SimpleNamespace(
                    source_run_dir=root,
                    return_dir=root / 'return',
                    deadline=time.time() + 300,
                    max_seconds=60,
                )
                live = {topic: list(gids) for topic, gids in meta['gids'].items()}
                live[changed] = ['new']
                probe = Mock()
                probe.ready.return_value = dict(gids=live)
                with (
                    patch.object(prefix, 'load_prefix', return_value=(config, source, meta, root)),
                    patch.object(prefix, 'Probe', return_value=probe),
                    patch.object(prefix, 'lock', side_effect=lambda _: nullcontext()),
                ):
                    with self.assertRaisesRegex(RuntimeError, 'GID differs'):
                        prefix.start(args)
                self.assertFalse(args.return_dir.exists())
                probe.close.assert_called_once()

    def test_command_is_fixed_slow_dual_epoch_and_manual(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            _config, source, meta, _module = fixture(root)
            op = prefix.PrefixReturnOperation(
                root,
                dict(max_seconds=60, deadline_unix=time.time() + 300, auto_next=False),
                Mock(),
                meta['gids'],
                source,
                root / 'points.json',
                root,
            )
            command = op.build_command()
            self.assertEqual(command[0], str(root / 'route_prefix_return_runner'))
            for option, expected in [
                ('--source-run-dir', str(root)),
                ('--confirm-same-odom', 'session'),
                ('--odom-publisher-gid', ODOM[0]),
                ('--raw-odom-publisher-gid', ODOM[1]),
                ('--speed', '0.03'),
                ('--turn-speed', '0.10'),
            ]:
                self.assertEqual(command[command.index(option) + 1], expected)
            self.assertIsNone(op.auto)
            self.assertIn('--laser-log-only', command)
            op.runner = SimpleNamespace(stdin=StringIO())
            rows = [
                dict(
                    event='return_session_started',
                    source_session=source.session_id,
                    source_hash=source.source_hash,
                    source_origin=asdict(source.origin),
                    publisher_gid=meta['gids'][ODOM[0]][0],
                )
            ]
            op.probe.snapshot.return_value = dict(scan_fresh=True, samples={SCAN: dict(stamp=100)})
            op.save_state(rows)
            self.assertEqual(op.runner.stdin.getvalue(), '')
            op.request(dict(id='one', command='next', wall_s=time.time()), rows)
            op.request(dict(id='two', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\n')
            op.cancelled = True
            op.busy = False
            op.state['ready'] = True
            op.request(dict(id='three', command='next', wall_s=time.time()), rows)
            self.assertEqual(op.runner.stdin.getvalue(), 'next\n')

    def test_start_records_prefix_evidence_and_runs_only_manual_operation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config, source, meta, _module = fixture(root)
            config['support'] = str(root)
            for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py'):
                (root / name).write_text(name)
            args = SimpleNamespace(
                source_run_dir=root,
                return_dir=root / 'return',
                deadline=time.time() + 300,
                max_seconds=60,
            )
            probe = Mock()
            probe.ready.return_value = dict(gids=meta['gids'])
            op = Mock()
            op.run.return_value = 2
            with (
                patch.object(prefix, 'load_prefix', return_value=(config, source, meta, root)),
                patch.object(prefix, 'Probe', return_value=probe),
                patch.object(prefix, 'lock', side_effect=lambda _: nullcontext()),
                patch.object(prefix, 'git', return_value='commit'),
                patch.object(prefix, 'PrefixReturnOperation', return_value=op) as operation,
            ):
                self.assertEqual(prefix.start(args), 2)
            options = operation.call_args.args[1]
            self.assertFalse(options['auto_next'])
            self.assertEqual(options['source_mode'], 'completed_stopped_prefix')
            manifest = json.loads((args.return_dir / 'manifest.json').read_text())
            self.assertEqual(manifest['prefix_evidence'], meta)
            self.assertFalse(manifest['task6_acceptance'])
            self.assertFalse(json.loads((root / 'result.json').read_text())['success'])
            probe.close.assert_called_once()
            op.run.assert_called_once()

    def test_auto_option_is_unavailable(self):
        with self.assertRaises(SystemExit) as error:
            prefix.main(['start', '--auto-next'])
        self.assertEqual(error.exception.code, 2)

    def test_deadline_refused_before_probe(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config, source, meta, _module = fixture(root)
            args = SimpleNamespace(
                source_run_dir=root,
                return_dir=root / 'return',
                deadline=time.time() + 100,
                max_seconds=60,
            )
            with (
                patch.object(prefix, 'load_prefix', return_value=(config, source, meta, root)),
                patch.object(prefix, 'Probe') as probe,
            ):
                with self.assertRaisesRegex(ValueError, 'deadline'):
                    prefix.start(args)
                probe.assert_not_called()


if __name__ == '__main__':
    unittest.main()
