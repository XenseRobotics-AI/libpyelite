"""Exercise Qt startup, including Python interpreters with file capabilities."""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


@unittest.skipUnless(
    sys.platform.startswith("linux") and importlib.util.find_spec("PySide6"),
    "Linux Qt runtime and optional simulation dependency",
)
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

    def test_child_cli_can_restart_after_its_parent(self):
        env = os.environ.copy()
        env["LIBPYELITE_QT_REEXEC_PID"] = str(os.getpid())
        env.pop("LD_LIBRARY_PATH", None)
        env["QT_QPA_PLATFORM"] = "offscreen"
        result = subprocess.run(
            [sys.executable, "-m", "libpyelite.sim", "--no-show", "--no-meshcat"],
            env=env, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_lost_library_path_after_restart_fails_instead_of_looping(self):
        # Simulate a launcher stripping the path on re-entry. A timeout here is
        # the original bug: the setup helper keeps exec'ing without reaching Qt.
        script = (
            "import os; "
            "os.environ.pop('LD_LIBRARY_PATH', None); "
            "os.environ['LIBPYELITE_QT_REEXEC_PID'] = str(os.getpid()); "
            "from libpyelite.sim import _prepare_qt_runtime; _prepare_qt_runtime()"
        )
        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True, text=True, timeout=10,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Refusing to restart again", result.stderr)


if __name__ == "__main__":
    unittest.main()
