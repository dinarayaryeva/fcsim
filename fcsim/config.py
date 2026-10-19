import copy
import yaml


DEFAULTS = {
    "run": {
        "seed": 1,          # Random seed
        "duration_s": 30.0, # Simulation duration (s)
    },
    "fc": {
        "sys_id": 1,                 # MAVLink system ID
        "comp_id": 1,                # MAVLink component ID
        "ready_after_s": 3.0,        # Startup time until ready (s)
        "setpoint_timeout_ms": 500,  # Setpoint timeout before hold (ms)
    },
    "vehicle": {
        "tau_s": 0.4,           # Response time constant (s)
        "vmax_mps": 15.0,        # Maximum speed (m/s)
        "kp_pos": 1.0,           # Position control gain
        "origin_lat": 55.0,      # Initial latitude
        "origin_lon": 37.0,      # Initial longitude
        "origin_alt_m": 150.0,   # Initial altitude (m)
        "noise_att_rad": 0.002,  # Attitude noise (rad)
        "noise_pos_m": 0.03,     # Position noise (m)
        "noise_vel_mps": 0.02,   # Velocity noise (m/s)
    },
    "battery": {
        "capacity_mah": 10000.0,  # Capacity (mAh)
        "cells": 6,               # Number of cells
        "r_int_ohm": 0.03,        # Internal resistance (ohm)
        "i_idle_a": 0.5,          # Idle current (A)
        "i_hover_a": 25.0,        # Hover current (A)
        "i_per_mps2_a": 0.05,     # Current per acceleration (A/(m/s²))
        "low_warn_pct": 20,       # Low-battery warning (%)
    },
    "link": {
        "bind": "127.0.0.1:14550",  # Local UDP address (real-time only)
        "peer": "127.0.0.1:14540",  # Remote UDP address (real-time only)
    },
}


def _merge(base, over, path=""):
    for k, v in over.items():
        if k not in base:
            raise ValueError(f"unknown parameter: {path}{k}")
        if isinstance(base[k], dict):
            _merge(base[k], v, f"{path}{k}.")
        else:
            base[k] = str(v) if isinstance(base[k], str) else type(base[k])(v)


def build(raw: dict) -> dict:
    cfg = copy.deepcopy(DEFAULTS)
    raw = dict(raw)
    scripts = raw.pop("script", [])
    faults = raw.pop("fault", [])
    _merge(cfg, raw)
    cfg["script"], cfg["fault"] = scripts, faults
    return cfg


def load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return build(raw)
