import os
import subprocess
import sys
from pathlib import Path


def test_real_qt_sliders_with_safety_and_home():
    root = Path(__file__).resolve().parents[3]
    script = r"""
import json,numpy as np
from libpyelite.sim import StationSimulation, make_window, _load_qt
from libpyelite.control_profile import load_recipe
qt=_load_qt();app=qt['QApplication']([])
settings=load_recipe('recipes/teleop/bi_elite_cs66_rt/diagonal-08-taccap.yaml')
settings.update(json.load(open('third_party/libpyelite/examples/sim_safety.json')))
s=StationSimulation(settings=settings,collision_profile='auto')
w=make_window(qt,s);w.render_timer.stop();home=s.q.copy()
control=w.left_pose.controls['x'];control.slider.setValue(control._to_position(control.value()+.1))
for _ in range(160): w.advance()
assert s.collision_error is None, s.collision_error
assert np.linalg.norm(s.tcp_poses()['left'][:3,3]-s.targets()['left'][:3,3])<.001,w.ik_status.text()
assert np.max(abs(s.q-home))>.1
button=w.findChild(qt['QPushButton'],'reset_to_home');assert button is not None
button.click();np.testing.assert_array_equal(s.q,home)
for side in s.arms:np.testing.assert_allclose(s.targets()[side],s.tcp_poses()[side])
assert w.render_timer.isActive()
# FK must not redefine the startup reference, including after a latched error.
q=home.copy();q[0]+=.01;s.reset(q);s.collision_error='injected pause';w.render_timer.stop()
button.click();np.testing.assert_array_equal(s.q,home)
assert s.collision_error is None and w.render_timer.isActive()
w.close()
"""
    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = "offscreen"
    # CLI normally configures Qt. This test imports the widget directly.
    import PySide6

    env["LD_LIBRARY_PATH"] = str(Path(PySide6.__file__).parent / "Qt/lib")
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=root, env=env, capture_output=True, text=True, timeout=60
    )
    assert result.returncode == 0, result.stdout + result.stderr
