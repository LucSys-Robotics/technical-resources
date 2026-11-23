# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

"""Verify that the notice adapter cannot accept missing or changed statements."""
from pathlib import Path
import tempfile
import unittest

from ament_copyright import UNKNOWN_IDENTIFIER
from ament_copyright.parser import parse_file

ROOT = Path(__file__).resolve().parents[1]


class NoticePolicy(unittest.TestCase):
    """Check positive and negative examples with the real ament parser."""

    def parse(self, content):
        """Parse a temporary source file using the installed registry."""
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'example.py'
            path.write_text(content)
            return parse_file(str(path))

    def test_valid_notice(self):
        """Accept only the registered maintenance notice."""
        content = (ROOT / 'launch/base_serial.launch.py').read_text()
        self.assertEqual(self.parse(content).license_identifier, 'lucsys_supplier_authorization')

    def test_missing_notice_rejected(self):
        """Missing statements must fail identification."""
        self.assertEqual(self.parse('print(1)\n').license_identifier, UNKNOWN_IDENTIFIER)

    def test_modified_notice_rejected(self):
        """Do not accept an accidental replacement license grant."""
        content = (ROOT / 'launch/base_serial.launch.py').read_text()
        content = content.replace('does not grant', 'grants')
        self.assertEqual(self.parse(content).license_identifier, UNKNOWN_IDENTIFIER)


if __name__ == '__main__':
    unittest.main()
