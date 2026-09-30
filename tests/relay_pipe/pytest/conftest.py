# SPDX-License-Identifier: GPL-3.0
"""Fixtures for the relay's two-image harness. The hub and the frame builder
are the attachment harness's (tests/attachment_pipe/pytest)."""

import os
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "attachment_pipe", "pytest"))

from hub import Hub  # noqa: E402

HALF = 0x0E0E0E0E   # the receiving half (on the bench, a MediumFast XIAO)
EAR = 0x00EA0001    # the ear (on the bench, a LongFast XIAO)


def pytest_addoption(parser):
    parser.addoption("--attach-build", action="store", default=None,
                     help="the sysbuild build directory holding both images")


def _opt(config, name):
    try:
        return config.getoption(name)
    except ValueError:
        return None


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
    half = next((d for d in os.listdir(build_dir)
                 if d not in ("ear", "_sysbuild", "zephyr")
                 and os.path.exists(os.path.join(build_dir, d, "zephyr", "zephyr.exe"))), None)
    ear = os.path.join(build_dir, "ear", "zephyr", "zephyr.exe")
    if half is None or not os.path.exists(ear):
        pytest.fail(f"missing images under {build_dir} (built with --sysbuild?)")
    return {"half": os.path.join(build_dir, half, "zephyr", "zephyr.exe"), "ear": ear}


@pytest.fixture
def hub(tmp_path, request):
    log = []
    h = Hub(str(tmp_path), log=log.append)
    yield h
    h.close()
    if getattr(request.node, "rep_call", None) is not None and request.node.rep_call.failed:
        print("\n".join(log))


@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)


def pair(hub, images, takes_env=False, lat=(20, 80)):
    """The bench pair: the half, the ear pointed at it, and the peer link. A
    relay half never says it takes envelopes (its beats carry no ATTACH)."""
    hub.start(images["half"], HALF, name="half")
    hub.start(images["ear"], EAR, name="ear", extra_env={"ATTACH_PIPE_PEER": f"{HALF:08x}"})
    hub.link(EAR, HALF, lat=lat, takes_env=takes_env)
    import time
    time.sleep(0.3)
