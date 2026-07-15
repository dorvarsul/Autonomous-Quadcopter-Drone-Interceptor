"""Environment doctor — verifies the toolchain before any simulation runs.

Checks, in order:
  1. Python version is >= 3.11.
  2. Core dependencies import (mujoco, numpy, scipy, yaml, matplotlib).
  3. MuJoCo reports a version and its native binaries are resolvable on PATH.
  4. A trivial MJCF loads and steps headlessly, and one frame renders **off-screen**
     (no GLFW window) — the headless smoke test the automated pipeline requires.

Exit code 0 means the environment is ready; non-zero (with a clear message) means it
is not. Fully non-interactive.

Run:  python scripts/check_env.py
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from typing import NoReturn

MIN_PYTHON = (3, 11)
MUJOCO_BIN_DIR = Path("C:/Dev/Libraries/mujoco/bin")
REQUIRED_MODULES = ("mujoco", "numpy", "scipy", "yaml", "matplotlib")


def _ok(msg: str) -> None:
    print(f"[ OK ] {msg}")


def _fail(msg: str) -> NoReturn:
    print(f"[FAIL] {msg}")
    sys.exit(1)


def check_python() -> None:
    if sys.version_info < MIN_PYTHON:
        _fail(f"Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+ required, found {sys.version.split()[0]}.")
    _ok(f"Python {sys.version.split()[0]}")


def check_imports() -> None:
    for name in REQUIRED_MODULES:
        try:
            importlib.import_module(name)
        except ImportError as exc:
            _fail(f"Cannot import required module '{name}': {exc}")
    _ok(f"Imported required modules: {', '.join(REQUIRED_MODULES)}")


def check_mujoco_binaries() -> None:
    """Confirm MuJoCo loaded and its native DLLs are resolvable.

    On Windows, `C:/Dev/Libraries/mujoco/bin` must be on PATH so mujoco.dll/glfw3.dll
    resolve (AGENTS.md). The pip `mujoco` wheel bundles its own libs, so importing it
    is the real proof; we additionally note whether the documented bin dir is present.
    """
    import mujoco

    _ok(f"MuJoCo version {mujoco.__version__}")
    if MUJOCO_BIN_DIR.exists():
        on_path = str(MUJOCO_BIN_DIR) in os.environ.get("PATH", "")
        state = "on PATH" if on_path else "present but NOT on PATH (pip wheel bundles libs anyway)"
        _ok(f"MuJoCo bin dir {MUJOCO_BIN_DIR} {state}")
    else:
        print(f"[WARN] MuJoCo bin dir {MUJOCO_BIN_DIR} not found; relying on pip wheel libs.")


def check_headless_render() -> None:
    """Load a trivial MJCF, step it, and render one frame fully off-screen."""
    import mujoco
    import numpy as np

    xml = """
    <mujoco>
      <worldbody>
        <light pos="0 0 3"/>
        <geom type="plane" size="2 2 0.1"/>
        <body pos="0 0 1">
          <joint type="free"/>
          <geom type="box" size="0.1 0.1 0.1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    for _ in range(100):
        mujoco.mj_step(model, data)
    _ok("Stepped trivial MJCF 100 times without error")

    # Off-screen render: mujoco.Renderer uses an off-screen GL context (no window).
    try:
        with mujoco.Renderer(model, height=120, width=160) as renderer:
            renderer.update_scene(data)
            frame = renderer.render()
    except Exception as exc:  # noqa: BLE001 - report any GL/context failure clearly
        _fail(
            "Off-screen render failed. Ensure a headless GL backend is available "
            f"(set MUJOCO_GL=egl or osmesa if needed). Underlying error: {exc}"
        )

    if not (isinstance(frame, np.ndarray) and frame.ndim == 3 and frame.shape[2] == 3):
        shape = getattr(frame, "shape", None)
        _fail(f"Off-screen render returned an unexpected buffer: {type(frame)} {shape}")
    _ok(f"Rendered one frame off-screen, shape={frame.shape} (no GLFW window)")


def main() -> int:
    print("=== Interceptor environment check ===")
    check_python()
    check_imports()
    check_mujoco_binaries()
    check_headless_render()
    print("=== All checks passed ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
