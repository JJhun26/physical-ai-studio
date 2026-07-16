"""Vendored third-party robot SDKs.

These are bundled rather than declared as dependencies so the backend stays
self-contained: nothing here reaches outside the studio tree at runtime.

``scservo_sdk``
    Feetech STS3215 servo SDK (``sms_sts`` variant), used by the uArm leader.
    Deliberately NOT the PyPI ``feetech-servo-sdk`` that ``lerobot[feetech]``
    already pulls in: that distribution ships a different, older module set
    (``packet_handler``) with no ``sms_sts`` and no ``SMS_STS_PRESENT_POSITION_L``,
    which the uArm leader needs. Both distributions claim the *same* top-level
    ``scservo_sdk`` name, so this copy lives inside the package and is imported as
    ``robots.drivers.vendor.scservo_sdk`` — never via ``sys.path``, which would
    make the winner depend on path order.
    Upstream: LeRobot-Anything-U-Arm (``Uarm_teleop/Feetech_servo/scservo_sdk``).
    Needs ``pyserial`` (declared in pyproject).

``fairino``
    Fairino FR5 controller SDK, used by the FR5 follower. Pure XML-RPC over port
    20003 — a single ``Robot.py``; ``ctypes`` appears only for struct layouts, no
    native library is loaded, so nothing outside this directory is required.
    Upstream: ``fairino-python-sdk-2.2.1_robot3.9.1/linux/fairino/Robot.py``.
    Version-locked to controller firmware v3.9.1: the newer v3.9.7/CNDE SDK
    rejects every call on our controller with error -4. Re-vendor only alongside
    a firmware upgrade.

Vendored code is excluded from ruff and pyrefly (see pyproject) — keep it
byte-identical to upstream so it can be re-vendored by copying.
"""
