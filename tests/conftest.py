"""
Pytest configuration for robot tests.

Forces process exit after test session completes to work around
photonlibpy's TimeSyncServer creating non-daemon threads that
prevent the process from exiting cleanly on Windows CI.

On GitHub Actions, each failing test also becomes an error annotation on
the run (and the PR), so the failure can be read without opening the log.
"""

import atexit
import os
import sys

_exit_code = 0


def pytest_sessionfinish(session, exitstatus):
    global _exit_code
    _exit_code = exitstatus


def pytest_runtest_logreport(report):
    """On GitHub Actions, turn a failed test into an ``::error`` annotation."""
    if not report.failed or os.environ.get("GITHUB_ACTIONS") != "true":
        return
    message = str(report.longreprtext).strip().splitlines()
    # Keep the end of the traceback, where the assert and its values are.
    text = "%0A".join(message[-15:]).replace("::", ": :")
    title = report.nodeid.replace("::", " ")
    print(f"\n::error title={title} ({report.when})::{text}", flush=True)


@atexit.register
def _force_exit():
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_exit_code)
