"""No-motion reference integrity and official scene4 entry contracts."""

import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import task8_reference as ref  # noqa: E402
import maze_entry  # noqa: E402
from scene_route import load_scene  # noqa: E402


@pytest.fixture
def evidence(tmp_path):
    route = ROOT / 'waypoints/waypoints_real.yaml'
    actions = load_scene(route, 2)['actions']
    book = tmp_path / 'forward.json'
    book.write_text(json.dumps({'frame': 'odom'}))
    events = [dict(event='route_started', scene=2, actions=actions)]
    events += [dict(event='route_action_completed', name=a['name']) for a in actions]
    events += [
        dict(event='route_completed', destination='P15', final_action='P15_after_turn', exit_code=0)
    ]
    book.with_suffix('.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    bag = tmp_path / 'evidence/bag/bag_0.db3'
    bag.parent.mkdir(parents=True)
    bag.write_bytes(b'synthetic integrity fixture, not a live laser reference')
    (bag.parent / 'metadata.yaml').write_text('synthetic: true\n')
    (bag.parent.parent / 'result.json').write_text(
        json.dumps(dict(closed_metadata=True, exception=None))
    )
    registry = tmp_path / 'registry/reference.json'
    return book, bag, route, registry


def test_roundtrip_registration_does_not_need_ros(evidence):
    book, bag, route, registry = evidence
    assert ref.register(book, bag, route, registry) == registry
    assert ref.resolve(route, registry) == (book.resolve(), bag.resolve())
    assert registry.stat().st_mode & 0o077 == 0


@pytest.mark.parametrize('role', ['book', 'events', 'bag', 'metadata', 'closure'])
def test_any_changed_evidence_rejects(evidence, role):
    book, bag, route, registry = evidence
    ref.register(book, bag, route, registry)
    path = ref.evidence_paths(book, bag)[role]
    path.write_bytes(path.read_bytes() + b'changed')
    with pytest.raises(ValueError, match='missing or changed'):
        ref.resolve(route, registry)


def test_missing_registry_rejects(evidence):
    _, _, route, registry = evidence
    with pytest.raises(ValueError, match='not registered'):
        ref.resolve(route, registry)


@pytest.mark.parametrize('bad', [[], {'schema': 2}, {'schema': 1, 'purpose': 'old_odom'}])
def test_invalid_schema_or_purpose_rejects(evidence, bad):
    _, _, route, registry = evidence
    registry.parent.mkdir()
    registry.write_text(json.dumps(bad))
    with pytest.raises(ValueError, match='Invalid'):
        ref.resolve(route, registry)


def test_unclosed_bag_is_not_registered(evidence):
    book, bag, route, registry = evidence
    (bag.parent.parent / 'result.json').write_text('{"closed_metadata":false}')
    with pytest.raises(ValueError, match='closed cleanly'):
        ref.register(book, bag, route, registry)
    assert not registry.exists()


def test_partial_forward_is_not_registered(evidence):
    book, bag, route, registry = evidence
    events = [json.loads(x) for x in book.with_suffix('.jsonl').read_text().splitlines()]
    events.pop(-2)
    book.with_suffix('.jsonl').write_text('\n'.join(json.dumps(x) for x in events))
    with pytest.raises(ValueError, match='incomplete'):
        ref.register(book, bag, route, registry)


def test_evidence_path_substitution_rejects(evidence):
    book, bag, route, registry = evidence
    ref.register(book, bag, route, registry)
    data = json.loads(registry.read_text())
    extra = registry.parent / 'other_metadata'
    extra.write_text('synthetic: true\n')
    data['files']['metadata']['path'] = str(extra)
    registry.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='paths disagree'):
        ref.resolve(route, registry)


def test_default_location_is_outside_package(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path))
    assert ref.default_registry() == tmp_path / 'pid_maze_solver/p15_reference.json'


def test_bare_scene4_resolves_reference_and_complete_route(evidence, monkeypatch):
    book, bag, route, registry = evidence
    ref.register(book, bag, route, registry)
    monkeypatch.setattr(ref, 'default_registry', lambda: registry)

    def backend(command):
        manifest = json.loads(Path(command[command.index('--route-json') + 1]).read_text())
        assert len(manifest['actions']) == 21
        assert manifest['actions'][-1]['name'] == 'P01_after_turn'
        assert command[command.index('--p15-start') + 1] == 'localized'
        assert (
            command[command.index('--p15-observation') + 1] == 'front_heading_fixed_reference_side'
        )
        assert command[command.index('--source-book') + 1] == str(book)
        return 0

    with (
        patch.object(maze_entry, 'executable', return_value='/fake'),
        patch.object(maze_entry, 'run_owned', side_effect=backend),
    ):
        assert maze_entry.main(['4']) == 0


def test_missing_default_reference_never_launches(evidence, monkeypatch):
    _, _, _, registry = evidence
    monkeypatch.setattr(ref, 'default_registry', lambda: registry)
    with patch.object(maze_entry, 'run_owned') as run:
        assert maze_entry.main(['4']) == 2
        run.assert_not_called()


def test_half_explicit_reference_rejects():
    with pytest.raises(SystemExit):
        maze_entry.parse_args(['4', '--source-book', '/only-book'])
