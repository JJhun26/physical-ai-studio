# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Calibration maths for the in-repo drivers.

Pure functions only -- no serial, no RPC, no DB. The headless
``scripts/calibrate_uarm.py`` and (later) an in-studio setup worker both build on
this module, so the mapping is derived and tested in exactly one place.
"""
