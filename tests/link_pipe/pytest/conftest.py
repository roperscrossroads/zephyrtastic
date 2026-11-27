# SPDX-License-Identifier: GPL-3.0
"""Fixtures for the two-image link harness.

Under twister (harness: pytest) the sysbuild build directory, which holds both
images, is the directory of --twister-config. Standalone:

    pytest tests/link_pipe/pytest/test_link_pipe.py --link-build=<build dir>
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from switchboard import Switchboard  # noqa: E402

HUB = 0x0A0A0A0A
WORKER1 = 0x00E10001
WORKER2 = 0x00E10002


def pytest_addoption(parser):
    parser.addoption("--link-build", action="store", default=None,
                     help="the sysbuild build directory holding both images")


def _opt(config, name):
    try:
        return config.getoption(name)
    except ValueError:
        return None  # an option of a plugin that is not loaded


def _exe(build_dir, image):
    path = os.path.join(build_dir, image, "zephyr", "zephyr.exe")
    if not os.path.exists(path):
        pytest.fail(f"missing image {path} (built with --sysbuild?)")
    return path


@pytest.fixture(scope="session")
def images(request):
    build_dir = (_opt(request.config, "--link-build") or os.environ.get("LINK_PIPE_BUILD")
                 or _opt(request.config, "--build-dir"))
    twister_cfg = _opt(request.config, "--twister-config")
    if not build_dir and twister_cfg:
        build_dir = os.path.dirname(twister_cfg)
    if not build_dir:
        pytest.fail("give --link-build=<sysbuild build dir> (or LINK_PIPE_BUILD)")
    build_dir = os.path.abspath(build_dir)
    # The default image is named after the application directory.
    hub = next((d for d in os.listdir(build_dir)
                  if d not in ("worker", "_sysbuild", "zephyr")
                  and os.path.exists(os.path.join(build_dir, d, "zephyr", "zephyr.exe"))), None)
    if hub is None:
        pytest.fail(f"no hub image under {build_dir}")
    return {"hub": _exe(build_dir, hub), "worker": _exe(build_dir, "worker")}


@pytest.fixture
def switchboard(tmp_path, request):
    log = []
    h = Switchboard(str(tmp_path), log=log.append)
    yield h
    h.close()
    if request.node.rep_call.failed if hasattr(request.node, "rep_call") else False:
        print("\n".join(log))


@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)


def start_hub(switchboard, images, preset=None):
    env = {} if preset is None else {"LINK_PIPE_PRESET": f"{preset:x}"}
    return switchboard.start(images["hub"], HUB, name="hub", extra_env=env)


def start_worker(switchboard, images, node, preset, hub=HUB):
    return switchboard.start(images["worker"], node, name=f"worker-{node:08x}",
                     extra_env={"LINK_PIPE_HUB": f"{hub:08x}",
                                "LINK_PIPE_PRESET": f"{preset:x}"})
