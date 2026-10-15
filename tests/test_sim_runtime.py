"""Exercise the CLI under the Conda/pip Qt library-path conflict."""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(importlib.util.find_spec("PySide6"), "optional simulation dependency")
class QtRuntimeTests(unittest.TestCase):
    def test_cli_with_conda_libraries_first(self):
        with tempfile.TemporaryDirectory() as cache:
            env = os.environ.copy()
            env["LD_LIBRARY_PATH"] = str(Path(sys.base_prefix) / "lib")
            env["QT_QPA_PLATFORM"] = "offscreen"
            env["QT_PLUGIN_PATH"] = "/nonexistent/conda/qt/plugins"
            env["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/nonexistent/conda/qt/platforms"
            env["XDG_CACHE_HOME"] = cache
            result = subprocess.run(
                [sys.executable, "-m", "libpyelite.sim", "--no-show", "--no-meshcat"],
                env=env, capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
