# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

"""Check the actual inverse-square-root function under memory sanitizers."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class InverseSquareRoot(unittest.TestCase):
    """Preserve the numerical algorithm without undefined pointer reads."""

    def test_positive_normal_inputs(self):
        """Check 10001 inputs across 60 orders of magnitude with ASan and UBSan."""
        source = (ROOT / 'src/Quaternion_Solution.cpp').read_text()
        function = re.search(r'float InvSqrt\(float number\)\s*\{.*?\n\}', source, re.S)
        self.assertIsNotNone(function)
        harness = r'''
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
FUNCTION
int main() {
  static_assert(sizeof(int16_t) == 2, "Serial fields must remain 16-bit");
  for (int i = 0; i <= 10000; ++i) {
    float input = std::pow(10.0f, -30.0f + 60.0f * i / 10000.0f);
    float actual = InvSqrt(input);
    float expected = 1.0f / std::sqrt(input);
    if (!std::isfinite(actual) || actual <= 0.0f ||
        std::abs(actual / expected - 1.0f) > 0.002f) {
      std::cerr << input << " " << actual << " " << expected << std::endl;
      return 1;
    }
  }
  return 0;
}
'''.replace('FUNCTION', function.group())
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / 'inverse_sqrt.cpp'
            executable = Path(folder) / 'inverse_sqrt'
            cpp.write_text(harness)
            subprocess.run([
                os.environ.get('CXX', 'c++'), '-std=c++14', '-O1',
                '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                str(cpp), '-o', str(executable),
            ], check=True, capture_output=True, text=True)
            subprocess.run([str(executable)], check=True, capture_output=True, text=True)


if __name__ == '__main__':
    unittest.main()
