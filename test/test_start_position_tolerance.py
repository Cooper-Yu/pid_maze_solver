"""Independent tolerance admission, manifest compatibility and full command projection."""

import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import maze_entry  # noqa: E402 - load the actual sibling tools tree
import task6_ops as ops  # noqa: E402
from task6_ops_runtime import Operation  # noqa: E402


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf', '.00499', '.01001'])
def test_all_cli_invalid_values(value, tmp_path):
    with pytest.raises(SystemExit):
        ops.parse_args(
            ['preflight', '--run-dir', str(tmp_path), '--start-position-tolerance=' + value]
        )
    with pytest.raises(SystemExit):
        maze_entry.parse_args(['2', '--start-position-tolerance=' + value])


@pytest.mark.parametrize('scene', [1, 3])
def test_other_scenes_do_not_accept_new_tolerance(scene):
    args, _ = maze_entry.parse_args([str(scene)])
    assert args.start_position_tolerance is None
    with pytest.raises(SystemExit):
        maze_entry.parse_args([str(scene), '--start-position-tolerance', '.010'])
    if scene == 3:
        with pytest.raises(SystemExit):
            maze_entry.parse_args(
                ['3', '--ros-args', '-p', 'precise_start_position_tolerance:=.010']
            )


def test_default_retained(tmp_path):
    assert (
        ops.parse_args(['preflight', '--run-dir', str(tmp_path)]).start_position_tolerance == 0.005
    )
    args, _ = maze_entry.parse_args(['2'])
    assert args.start_position_tolerance == 0.005


def prepared(tmp_path, tolerance=0.010, corner_placement='off'):
    entry = tmp_path / 'pid_maze_solver'
    entry.write_text('fixture, never execute')
    entry.chmod(0o700)
    support = tmp_path / 'tools/task6_ops'
    support.mkdir(parents=True)
    for name in ('common.py', 'evidence_recorder.py', 'odom_reset_watch.py'):
        (support / name).write_text('# independent inert fixture')
    packages = {
        'pid_maze_solver': dict(source=str(ROOT), share=str(ROOT), library=str(tmp_path)),
        'distance_controller': dict(source=str(tmp_path)),
    }
    args = SimpleNamespace(
        workspace=tmp_path,
        deadline=time.time() + 1000,
        max_seconds=300,
        speed=0.04,
        turn_speed=0.1,
        destination='P12',
        run_dir=tmp_path / 'run',
        start_position_tolerance=tolerance,
        corner_placement=corner_placement,
    )
    with patch.object(ops, 'provenance', return_value=packages):
        assert ops.prepare(args) == 0
    return args.run_dir, packages


def test_one_centimeter_persists_and_reaches_backend_command(tmp_path):
    directory, packages = prepared(tmp_path)
    with patch.object(ops, 'provenance', return_value=packages):
        config = ops.verify_config(directory)
    assert config['start_position_tolerance'] == 0.010
    operation = Operation.__new__(Operation)
    operation.directory, operation.config = directory, config
    command = operation.build_command()
    assert command[command.index('--start-position-tolerance') + 1] == '0.01'
    args, ros_args = maze_entry.parse_args(command[1:])
    assert args.scene == 2 and args.start_position_tolerance == 0.010 and not ros_args
    with patch.object(maze_entry, 'executable', return_value='/fixture/task6_route_runner'):
        backend = maze_entry.backend_command(args, directory / 'route.json', ros_args)
    assert backend[backend.index('--start-position-tolerance') + 1] == '0.01'
    assert '--ros-args' not in backend


def test_legacy_manifest_defaults_in_memory_only(tmp_path):
    directory, packages = prepared(tmp_path)
    file = directory / 'config.json'
    data = json.loads(file.read_text())
    del data['start_position_tolerance']
    file.write_text(json.dumps(data))
    before = file.read_bytes()
    with patch.object(ops, 'provenance', return_value=packages):
        config = ops.verify_config(directory)
    assert config['start_position_tolerance'] == 0.005
    assert file.read_bytes() == before
    operation = Operation.__new__(Operation)
    operation.directory, operation.config = directory, data
    command = operation.build_command()
    assert command[command.index('--start-position-tolerance') + 1] == '0.005'


@pytest.mark.parametrize('value', [True, None, '0.01', float('nan'), float('inf'), 0.004, 0.011])
def test_manifest_invalid_value_refused(tmp_path, value):
    directory, packages = prepared(tmp_path)
    file = directory / 'config.json'
    data = json.loads(file.read_text())
    data['start_position_tolerance'] = value
    file.write_text(json.dumps(data))
    with patch.object(ops, 'provenance', return_value=packages), pytest.raises(RuntimeError):
        ops.verify_config(directory)
