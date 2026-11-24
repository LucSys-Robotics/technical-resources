import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sync', ROOT / 'tools/sync_robot_resources.py')
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


class ResourceTests(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((ROOT / 'tools/robot_resources_manifest.json').read_text())

    def test_all_models_and_release_archives(self):
        entries = self.manifest['entries']
        self.assertEqual(len(self.manifest['models']), 10)
        for model in self.manifest['models']:
            cats = {e['category'] for e in entries if e['model'] == model}
            self.assertTrue({'01_STM32_Firmware', '02_ROS2_Driver', '03_URDF_Mesh', '05_Manuals'} <= cats)
        assets = {e['source'] for e in entries if e['source_kind'] == 'release'}
        self.assertEqual(len(assets), 34)
        self.assertEqual(sum('_description.zip' in a for a in assets), 26)

    def test_lsr550a_isolation(self):
        entries = [e for e in self.manifest['entries'] if e['model'] == 'LSR550A']
        firmware = [e for e in entries if e['category'] == '01_STM32_Firmware']
        self.assertEqual(len(firmware), 3)
        self.assertTrue(all(e['source'].startswith('lucsys_lsr550a_v') for e in firmware))
        electrical = [e for e in entries if e['category'] == '04_Electrical']
        self.assertEqual([e['source'] for e in electrical], ['electrical/LSR550A/LSR550A_C50C_Controller_Schematic.pdf'])

    def test_no_unconfirmed_schematics(self):
        self.assertFalse(any(e['model'] in ['LSR100', 'LSR200Auto'] and e['category'] == '04_Electrical' for e in self.manifest['entries']))

    def test_real_files_match_pinned_sources(self):
        for e in self.manifest['entries']:
            with self.subTest(target=e['target']):
                sync.verify(ROOT / e['target'], e)
                if e['source_kind'] == 'repository':
                    self.assertEqual((ROOT / e['target']).read_bytes(), (ROOT / e['source']).read_bytes())

    def test_duplicate_target_rejected(self):
        m = copy.deepcopy(self.manifest)
        m['entries'].append(m['entries'][0])
        with self.assertRaises(ValueError): sync.validate_manifest(m)

    def test_path_traversal_rejected(self):
        with self.assertRaises(ValueError): sync.safe_path(ROOT, '../escape')

    def test_mismatched_model_rejected(self):
        m = copy.deepcopy(self.manifest)
        m['entries'][0]['model'] = 'LSR550A'
        with self.assertRaises(ValueError): sync.validate_manifest(m)

    def test_sync_idempotence_and_modified_copy_protection(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source = root / 'manuals/test.pdf'
            source.parent.mkdir()
            source.write_bytes(b'original test fixture')
            e = dict(self.manifest['entries'][0], source_kind='repository', source='manuals/test.pdf', target='Robot-Resources/LSR100/05_Manuals/test.pdf', model='LSR100', category='05_Manuals', size=source.stat().st_size, sha256=hashlib.sha256(source.read_bytes()).hexdigest(), source_url='https://raw.githubusercontent.com/LucSys-Robotics/technical-resources/'+self.manifest['repository_commit']+'/manuals/test.pdf')
            m = dict(self.manifest, entries=[e])
            sync.run(root, m, 'sync', root)
            target = root / e['target']; before = target.stat().st_mtime_ns
            sync.run(root, m, 'sync', root)
            self.assertEqual(before, target.stat().st_mtime_ns)
            target.write_bytes(b'changed')
            with self.assertRaises(ValueError): sync.run(root, m, 'sync', root)
            self.assertEqual(target.read_bytes(), b'changed')

    def test_corrupt_source_rejected_before_copy(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            e = dict(self.manifest['entries'][0])
            (root / e['source']).write_bytes(b'bad archive')
            with self.assertRaises(ValueError): sync.run(root, dict(self.manifest, entries=[e]), 'sync', root)
            self.assertFalse((root / e['target']).exists())

    def test_unmanaged_files_preserved_and_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); p = root / 'Robot-Resources/unmanaged.txt'
            p.parent.mkdir(); p.write_text('keep')
            with self.assertRaises(ValueError): sync.run(root, self.manifest, 'sync', root)
            self.assertEqual(p.read_text(), 'keep')


if __name__ == '__main__': unittest.main()
