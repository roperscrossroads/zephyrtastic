# SPDX-License-Identifier: GPL-3.0
"""Fixtures for the two-image attachment harness.

Under twister (harness: pytest) the sysbuild build directory, which holds both
images, is the directory of --twister-config. Standalone:

    pytest tests/attachment_pipe/pytest/test_attach_pipe.py --attach-build=<build dir>
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from hub import Hub  # noqa: E402

BRAIN = 0x0A0A0A0A
HEAD1 = 0x00E10001
HEAD2 = 0x00E10002


def pytest_addoption(parser):
    parser.addoption("--attach-build", action="store", default=None,
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
    build_dir = (_opt(request.config, "--attach-build") or os.environ.get("ATTACH_PIPE_BUILD")
                 or _opt(request.config, "--build-dir"))
    twister_cfg = _opt(request.config, "--twister-config")
    if not build_dir and twister_cfg:
        build_dir = os.path.dirname(twister_cfg)
    if not build_dir:
        pytest.fail("give --attach-build=<sysbuild build dir> (or ATTACH_PIPE_BUILD)")
    build_dir = os.path.abspath(build_dir)
    # The default image is named after the application directory.
    brain = next((d for d in os.listdir(build_dir)
                  if d not in ("head", "_sysbuild", "zephyr")
                  and os.path.exists(os.path.join(build_dir, d, "zephyr", "zephyr.exe"))), None)
    if brain is None:
        pytest.fail(f"no brain image under {build_dir}")
    return {"brain": _exe(build_dir, brain), "head": _exe(build_dir, "head")}


@pytest.fixture
def hub(tmp_path, request):
    log = []
    h = Hub(str(tmp_path), log=log.append)
    yield h
    h.close()
    if request.node.rep_call.failed if hasattr(request.node, "rep_call") else False:
        print("\n".join(log))


@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)


def start_brain(hub, images):
    return hub.start(images["brain"], BRAIN, name="brain")


def start_head(hub, images, node, preset, brain=BRAIN):
    return hub.start(images["head"], node, name=f"head-{node:08x}",
                     extra_env={"ATTACH_PIPE_BRAIN": f"{brain:08x}",
                                "ATTACH_PIPE_PRESET": f"{preset:x}"})
