"""Build customer archives from explicit source roots, excluding compatibility/history."""
from pathlib import Path
import hashlib
import json
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=True)
rows = []
for asset in json.loads((Path(__file__).with_name('customer-assets.json')).read_text()):
    base = ROOT / asset['source']
    prefix = 'LSR550A/' + base.name if '/models/LSR550A/' in asset['source'] else Path(asset['file']).stem
    archive = output / asset['file']
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
        for source in sorted(base.rglob('*')):
            if not source.is_file() or any(x in source.parts for x in ('__pycache__', '.pytest_cache')):
                continue
            if source.suffix.lower() in ('.pyc', '.o', '.axf', '.hex', '.bin'):
                raise ValueError('Unexpected build output: ' + str(source))
            z.write(source, prefix + '/' + str(source.relative_to(base)))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert z.read(name) == (base / name[len(prefix) + 1:]).read_bytes()
        count = len(z.namelist())
    rows.append(dict(asset, sha256=hashlib.sha256(archive.read_bytes()).hexdigest(), files=count))
(output / 'manifest.json').write_text(json.dumps(rows, indent=2) + '\n')
(output / 'SHA256SUMS.txt').write_text(''.join(x['sha256'] + '  ' + x['file'] + '\n' for x in rows))
print('Verified customer archives:', len(rows))
