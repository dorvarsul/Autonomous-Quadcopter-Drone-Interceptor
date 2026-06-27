"""Per-run structured logging and reproducibility snapshots.

Two responsibilities, both serving determinism/reproducibility (AGENTS.md):

1. :class:`RunLogger` writes one structured CSV row per timestep, so any run can be
   replayed/plotted offline.
2. :func:`write_run_snapshot` records *everything needed to reproduce the run* — the
   resolved params, the seed, the git commit hash, and free-form metadata — into
   ``results/<run_id>/run_config.json``.

A run is identified by ``run_id``; all of its artifacts live under
``results/<run_id>/``. The logger is deterministic: given the same inputs it produces a
byte-identical CSV (stable column order, fixed float formatting, ``\\n`` newlines).
"""

from __future__ import annotations

import csv
import json
import subprocess
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

# Repo-root-relative default results directory.
DEFAULT_RESULTS_DIR = Path("results")

# Fixed float formatting so identical numbers serialize identically across machines.
_FLOAT_FORMAT = "{:.10g}"


def get_git_hash() -> str:
    """Return the current git commit hash, or ``"unknown"`` if not in a git repo.

    Recorded in the snapshot so a result is traceable to exact source. Never raises —
    a missing git is not a reason to abort a run, but it is worth flagging in the log.
    """
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def _format_value(value: Any) -> str:
    """Deterministic stringification for CSV cells."""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return _FLOAT_FORMAT.format(value)
    return str(value)


class RunLogger:
    """Append-only, deterministic per-timestep CSV logger.

    Columns are fixed at construction from ``fieldnames`` to guarantee stable ordering.
    Rows must provide exactly those fields; a missing/extra field fails loud so a
    silent schema drift cannot corrupt downstream analysis.
    """

    def __init__(
        self, run_dir: Path, fieldnames: Sequence[str], filename: str = "run_log.csv"
    ) -> None:
        self._fieldnames = list(fieldnames)
        run_dir.mkdir(parents=True, exist_ok=True)
        self._path = run_dir / filename
        # newline="" + explicit \n keeps line endings identical across OSes.
        self._handle = self._path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._handle, lineterminator="\n")
        self._writer.writerow(self._fieldnames)

    @property
    def path(self) -> Path:
        return self._path

    def log_step(self, row: Mapping[str, Any]) -> None:
        """Write one timestep row; the keys must match ``fieldnames`` exactly."""
        if set(row.keys()) != set(self._fieldnames):
            raise KeyError(
                f"Log row keys {sorted(row.keys())} do not match the declared "
                f"columns {sorted(self._fieldnames)}."
            )
        self._writer.writerow([_format_value(row[name]) for name in self._fieldnames])

    def close(self) -> None:
        self._handle.close()

    def __enter__(self) -> RunLogger:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def write_run_snapshot(
    run_dir: Path,
    *,
    seed: int,
    params: Mapping[str, Any],
    metadata: Mapping[str, Any] | None = None,
) -> Path:
    """Write the reproducibility snapshot to ``run_dir/run_config.json`` and return it.

    Captures the seed, the git hash, the resolved params, and any extra metadata
    (scenario name, step count, loop rates...). JSON is written with sorted keys and a
    trailing newline so identical inputs yield a byte-identical file.
    """
    run_dir.mkdir(parents=True, exist_ok=True)
    snapshot = {
        "seed": int(seed),
        "git_hash": get_git_hash(),
        "params": dict(params),
        "metadata": dict(metadata or {}),
    }
    path = run_dir / "run_config.json"
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(snapshot, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return path
