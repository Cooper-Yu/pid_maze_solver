#!/usr/bin/env python3
"""Register and verify recorded P15 wall references; never create a ROS publisher."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

from scene_route import load_scene
from task8_p15_start import validate_source


def default_registry():
    root = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
    return root / 'pid_maze_solver/p15_reference.json'


def endpoint_from_route(route):
    forward = load_scene(route, 2)
    return next(a for a in forward['actions'] if a['name'] == 'P15')['points'][-1]


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def evidence_paths(book, bag):
    book, bag = Path(book).resolve(strict=True), Path(bag).resolve(strict=True)
    return dict(
        book=book,
        events=book.with_suffix('.jsonl'),
        bag=bag,
        metadata=bag.parent / 'metadata.yaml',
        closure=bag.parent.parent / 'result.json',
    )


def register(book, bag, forward_route, registry=None):
    paths = evidence_paths(book, bag)
    validate_source(paths['book'], paths['bag'], endpoint_from_route(forward_route))
    data = dict(
        schema=1,
        purpose='p15_wall_reference_only',
        files={role: dict(path=str(path), sha256=digest(path)) for role, path in paths.items()},
    )
    target = Path(registry) if registry is not None else default_registry()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=target.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return target


def resolve(forward_route, registry=None):
    target = Path(registry) if registry is not None else default_registry()
    if not target.is_file():
        raise ValueError(
            'P15 reference not registered; run task8_reference once with closed evidence'
        )
    if target.stat().st_size > 65536:
        raise ValueError('P15 reference registry too large')
    data = json.loads(target.read_text())
    if (
        not isinstance(data, dict)
        or data.get('schema') != 1
        or data.get('purpose') != 'p15_wall_reference_only'
    ):
        raise ValueError('Invalid P15 reference registry')
    files = data.get('files')
    if not isinstance(files, dict) or set(files) != {
        'book',
        'events',
        'bag',
        'metadata',
        'closure',
    }:
        raise ValueError('Incomplete P15 reference registry')
    checked = {}
    for role, item in files.items():
        if not isinstance(item, dict) or not isinstance(item.get('path'), str):
            raise ValueError('Invalid P15 reference file entry')
        path = Path(item['path'])
        if not path.is_absolute() or not path.is_file() or digest(path) != item.get('sha256'):
            raise ValueError(f'P15 reference missing or changed: {role}')
        checked[role] = path.resolve(strict=True)
    if checked != evidence_paths(checked['book'], checked['bag']):
        raise ValueError('P15 reference evidence paths disagree')
    validate_source(checked['book'], checked['bag'], endpoint_from_route(forward_route))
    return checked['book'], checked['bag']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-book', type=Path, required=True)
    parser.add_argument('--source-bag', type=Path, required=True)
    parser.add_argument('--forward-route', type=Path, required=True)
    parser.add_argument('--registry', type=Path)
    args = parser.parse_args(argv)
    try:
        target = register(args.source_book, args.source_bag, args.forward_route, args.registry)
        print('P15_REFERENCE_REGISTERED_NO_MOTION:', target)
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('P15_REFERENCE_REJECTED:', error)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
