#!/usr/bin/env python3
"""Generate byte-identical customer copies from the pinned resource manifest.

Preview: python3 tools/sync_robot_resources.py
Sync:    python3 tools/sync_robot_resources.py --sync --cache /path/to/release/assets
Verify:  python3 tools/sync_robot_resources.py --check

Without --cache, release files are downloaded into a temporary directory.
Repository sources must match their pinned SHA256; this tool never edits them.
To update a resource, review its applicability and update the manifest explicitly.
Unexpected customer files or modified copies cause an error; nothing is deleted.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'tools/robot_resources_manifest.json'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def safe_path(root, relative):
    part = PurePosixPath(relative)
    if part.is_absolute() or '..' in part.parts:
        raise ValueError(f'Unsafe path: {relative}')
    path = root / relative
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f'Path escapes root: {relative}')
    return path


def validate_manifest(manifest):
    targets = set()
    for entry in manifest['entries']:
        target = entry['target']
        if target in targets:
            raise ValueError(f'Duplicate target: {target}')
        targets.add(target)
        if not target.startswith(f"Robot-Resources/{entry['model']}/{entry['category']}/"):
            raise ValueError(f'Model/category mismatch: {target}')
        safe_path(ROOT, target)
        if entry['model'] not in manifest['models'] or entry['size'] >= 100 * 1024**2:
            raise ValueError(f'Unknown model or oversized file: {target}')
        if len(entry['sha256']) != 64 or not entry['scope'] or not entry['evidence']:
            raise ValueError(f'Missing provenance: {target}')
        if entry['source_kind'] == 'release':
            prefix = 'https://github.com/LucSys-Robotics/technical-resources/releases/download/'
            if entry['source_url'] != prefix + manifest['release_tag'] + '/' + entry['source']:
                raise ValueError(f'Unpinned release: {target}')
            if PurePosixPath(entry['source']).name != entry['source']:
                raise ValueError('Invalid asset name')
        elif entry['source_kind'] == 'repository':
            safe_path(ROOT, entry['source'])
            expected = 'https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/'
            if entry['source_url'] != expected + manifest['repository_commit'] + '/' + entry['source']:
                raise ValueError(f'Unpinned repository source: {target}')
        else:
            raise ValueError('Unknown source kind')
    return targets


def verify(path, entry):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'Missing real file: {path}')
    if path.stat().st_size != entry['size'] or digest(path) != entry['sha256']:
        raise ValueError(f'Size/SHA256 mismatch: {path}')
    if path.suffix.lower() == '.zip':
        with zipfile.ZipFile(path) as archive:
            bad = archive.testzip()
            if bad:
                raise ValueError(f'Invalid ZIP member: {bad}')


def run(root, manifest, mode, cache):
    targets = validate_manifest(manifest)
    folder = root / 'Robot-Resources'
    existing = {str(p.relative_to(root)) for p in folder.rglob('*') if p.is_file() or p.is_symlink()}
    if existing - targets:
        raise ValueError(f'Unmanaged customer files: {sorted(existing - targets)}')
    # Preflight all existing copies and sources before writing anything.
    sources = {}
    verified = set()
    for entry in manifest['entries']:
        target = safe_path(root, entry['target'])
        if target.exists() or target.is_symlink():
            verify(target, entry)
        elif mode == 'check':
            raise ValueError(f'Missing customer file: {target}')
        if entry['source_kind'] == 'repository':
            source = safe_path(root, entry['source'])
        elif mode == 'sync':
            source = safe_path(cache, entry['source'])
            if not source.exists():
                with urllib.request.urlopen(entry['source_url'], timeout=120) as response:
                    with source.open('wb') as output:
                        shutil.copyfileobj(response, output)
        else:
            continue
        key = (str(source), entry['sha256'])
        if key not in verified:
            verify(source, entry)
            verified.add(key)
        sources[entry['target']] = source
    if mode == 'sync':
        for entry in manifest['entries']:
            target = safe_path(root, entry['target'])
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temp:
                    temp_path = Path(temp.name)
                try:
                    shutil.copyfile(sources[entry['target']], temp_path)
                    verify(temp_path, entry)
                    temp_path.replace(target)
                finally:
                    temp_path.unlink(missing_ok=True)
    return {'mode': mode, 'files': len(targets),
            'logical_bytes': sum(e['size'] for e in manifest['entries']),
            'unique_bytes': sum({e['sha256']: e['size'] for e in manifest['entries']}.values())}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--sync', action='store_true')
    modes.add_argument('--check', action='store_true')
    parser.add_argument('--cache', type=Path)
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    with tempfile.TemporaryDirectory(prefix='lucsys-resources-') as temporary:
        cache = args.cache or Path(temporary)
        cache.mkdir(parents=True, exist_ok=True)
        print(json.dumps(run(ROOT, manifest, 'sync' if args.sync else 'check' if args.check else 'preview', cache), indent=2))


if __name__ == '__main__':
    main()
