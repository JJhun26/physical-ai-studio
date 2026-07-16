# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Guards that the FR5/uArm drivers stay self-contained.

Their SDKs are vendored under ``robots/drivers/vendor`` rather than installed, so
the usual dependency machinery cannot catch a regression here. Nothing outside the
studio tree may be needed to import or default-construct a driver: an absolute path
to a checkout on one developer's machine imports fine there and raises ImportError
everywhere else.
"""

import ast
from pathlib import Path

import pytest

import robots.drivers as drivers_pkg
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig
from robots.drivers.uarm_leader import UArmLeaderClient, UArmLeaderConfig

DRIVERS_DIR = Path(drivers_pkg.__file__).parent
VENDOR_DIR = DRIVERS_DIR / "vendor"

# Driver modules we own; the vendored SDKs are upstream copies and exempt.
OWN_MODULES = sorted(p for p in DRIVERS_DIR.rglob("*.py") if VENDOR_DIR not in p.parents)


def _string_constants(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    return [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)]


@pytest.mark.parametrize("path", OWN_MODULES, ids=lambda p: p.name)
def test_no_absolute_path_defaults(path: Path) -> None:
    """No driver hard-codes a filesystem path outside the studio tree.

    Docstrings may still *name* an upstream checkout for provenance; only paths
    used as values are a dependency.
    """
    prefixes = ("/home/", "/Users/", "/opt/", "C:\\")
    offenders = [s for s in _string_constants(path) if s.startswith(prefixes)]
    assert offenders == [], f"{path.name} hard-codes an out-of-tree path: {offenders}"


def test_vendored_sdks_import_without_sys_path() -> None:
    """The vendored SDKs import as ordinary subpackages.

    Importing them via ``sys.path`` instead would be silently wrong: the PyPI
    ``feetech-servo-sdk`` that ``lerobot[feetech]`` installs claims the same
    top-level ``scservo_sdk`` name but ships no ``sms_sts``, so which one wins
    would depend on path order.
    """
    from robots.drivers.vendor.fairino import Robot
    from robots.drivers.vendor.scservo_sdk import SMS_STS_PRESENT_POSITION_L, sms_sts

    assert hasattr(Robot, "RPC")
    assert sms_sts.__module__.startswith("robots.drivers.vendor.")
    assert isinstance(SMS_STS_PRESENT_POSITION_L, int)


def test_vendored_sdk_is_pure_python() -> None:
    """The vendored tree carries no compiled artifacts.

    ``fairino`` uses ctypes only for struct layouts; if a ``.so``/``.pyd`` ever
    appears here, the copy is no longer portable and the upstream 71MB
    ``libfairino`` C++ SDK has crept in.
    """
    binaries = [p.name for p in VENDOR_DIR.rglob("*") if p.suffix in {".so", ".pyd", ".dll", ".dylib"}]
    assert binaries == [], f"vendored SDK contains native binaries: {binaries}"


@pytest.mark.parametrize(
    ("config", "client"),
    [(FR5FollowerConfig, FR5FollowerClient), (UArmLeaderConfig, UArmLeaderClient)],
)
def test_default_config_uses_vendored_sdk(config: type, client: type, monkeypatch: pytest.MonkeyPatch) -> None:
    """With no env override, drivers default to the vendored SDK.

    The override env vars are an escape hatch (e.g. an FR5 firmware upgrade needing
    a different SDK); they must not be required for a stock checkout to work.
    """
    monkeypatch.delenv("FAIRINO_SDK_LINUX", raising=False)
    monkeypatch.delenv("UARM_FEETECH_SDK", raising=False)

    assert config().sdk_path == ""
    # Resolves the import through the vendored package; no hardware contact.
    assert client(config())._import_sdk() is not None
