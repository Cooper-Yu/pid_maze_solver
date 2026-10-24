"""Bounded frontend option projection only; never execute the generated commands."""

import json
from unittest.mock import patch

import pytest

from test_start_position_tolerance import Operation, maze_entry, ops, prepared


def test_defaults_off(tmp_path):
    assert ops.parse_args(['preflight', '--run-dir', str(tmp_path)]).corner_placement == 'off'
    assert maze_entry.parse_args(['2'])[0].corner_placement == 'off'


@pytest.mark.parametrize('scene', ['1', '3'])
@pytest.mark.parametrize('value', ['off', 'front', 'lidar'])
def test_other_scenes_reject_explicit_option(scene, value):
    assert maze_entry.parse_args([scene])[0].corner_placement is None
    with pytest.raises(SystemExit):
        maze_entry.parse_args([scene, '--corner-placement', value])


def test_invalid_cli(tmp_path):
    with pytest.raises(SystemExit):
        ops.parse_args(['preflight', '--run-dir', str(tmp_path), '--corner-placement', 'shell'])
    with pytest.raises(SystemExit):
        maze_entry.parse_args(['2', '--corner-placement', 'shell'])


@pytest.mark.parametrize('mode', ['front', 'lidar'])
def test_manifest_to_backend_and_policy_metadata(tmp_path, mode):
    directory, packages = prepared(tmp_path, corner_placement=mode)
    with patch.object(ops, 'provenance', return_value=packages):
        config = ops.verify_config(directory)
    assert config['corner_placement'] == mode
    assert config['laser_policy'] == 'observation_only'
    assert config['start_position_tolerance'] == 0.010
    operation = Operation.__new__(Operation)
    operation.config, operation.directory = config, directory
    command = operation.build_command()
    assert command[command.index('--corner-placement') + 1] == mode
    args, ros_args = maze_entry.parse_args(command[1:])
    with patch.object(maze_entry, 'executable', return_value='/fixture/runner'):
        backend = maze_entry.backend_command(args, directory / 'route.json', ros_args)
    assert backend[backend.index('--corner-placement') + 1] == mode
    assert '--laser-log-only' in backend


def test_old_manifest_defaults_without_rewriting(tmp_path):
    directory, packages = prepared(tmp_path)
    file = directory / 'config.json'
    data = json.loads(file.read_text())
    del data['corner_placement']
    file.write_text(json.dumps(data))
    before = file.read_bytes()
    with patch.object(ops, 'provenance', return_value=packages):
        config = ops.verify_config(directory)
    assert config['corner_placement'] == 'off'
    assert file.read_bytes() == before
    operation = Operation.__new__(Operation)
    operation.config, operation.directory = data, directory
    command = operation.build_command()
    assert command[command.index('--corner-placement') + 1] == 'off'


@pytest.mark.parametrize('value', [None, True, 1, 'shell', [], {}])
def test_invalid_manifest(tmp_path, value):
    directory, packages = prepared(tmp_path)
    file = directory / 'config.json'
    data = json.loads(file.read_text())
    data['corner_placement'] = value
    file.write_text(json.dumps(data))
    with patch.object(ops, 'provenance', return_value=packages), pytest.raises(RuntimeError):
        ops.verify_config(directory)
