import copy
import math
from collections import namedtuple

from pymavlink.dialects.v20 import common as mav2
from fcsim import config
from fcsim.sim import Sim

Rx = namedtuple("Rx", "t_us name seq msg")
R = 6378137.0


def decode(frames):
    """Decode MAVLink frames and count invalid packets."""
    parser = mav2.MAVLink(None)
    parser.robust_parsing = True
    out, bad = [], 0

    for t, data in frames:
        for msg in parser.parse_buffer(data) or []:
            if msg.get_type() == "BAD_DATA":
                bad += 1
            else:
                out.append(Rx(t, msg.get_type(), msg.get_seq(), msg))

    return out, bad


def base_cfg(faults=(), duration=50.0, seed=7, **over):
    """Configure arming, setpoints, external mode, and optional overrides."""
    raw = {
        "run": {"seed": seed, "duration_s": duration},
        "script": [
            {"t_s": 5.0, "do": "arm"},
            {"t_s": 5.5, "do": "setpoint_stream", "kind": "vel",
             "vx": 5.0, "rate_hz": 20, "until_s": duration},
            {"t_s": 6.0, "do": "mode", "mode": "EXTERNAL"},
        ],
        "fault": list(faults),
    }
    for k, v in over.items():
        raw.setdefault(k, {}).update(v)
    return config.build(copy.deepcopy(raw))


def run(cfg):
    """Run the simulation and return telemetry and invalid-packet count."""
    sim = Sim(cfg)
    sim.run()
    rx, bad = decode(sim.transport.frames)
    return sim, rx, bad


def of(rx, name):
    return [r for r in rx if r.name == name]


def window(rx, t0_s, t1_s):
    return [r for r in rx if t0_s * 1e6 <= r.t_us < t1_s * 1e6]


def seq_anomalies(rx):
    return sum(
        r.seq != (a.seq + 1) & 0xFF
        for a, r in zip(rx, rx[1:])
    )


def time_regress(rx):
    last, n = {}, 0
    for r in rx:
        tb = getattr(r.msg, "time_boot_ms", None)
        if tb is not None:
            n += r.name in last and tb < last[r.name]
            last[r.name] = tb
    return n


def _frozen(values, k):
    return any(len(set(values[i:i + k])) == 1
               for i in range(len(values) - k + 1))


def ts_frozen(rx, name="ATTITUDE", k=3):
    return _frozen([r.msg.time_boot_ms for r in of(rx, name)], k)


def repeated_values(rx, name="ATTITUDE",
                    fields=("roll", "pitch", "yaw"), k=5):
    return _frozen([
        tuple(getattr(r.msg, f) for f in fields)
        for r in of(rx, name)
    ], k)


def _local_xy(m, lat0):
    return (
        (m.lat / 1e7 - lat0) * math.pi / 180 * R,
        m.lon / 1e7 * math.pi / 180 * R * math.cos(math.radians(lat0)),
    )


def pos_vel_residual(rx, lat0, win=50):
    """Return maximum position-versus-velocity mismatch."""
    g = of(rx, "GLOBAL_POSITION_INT")
    worst = 0.0

    for i in range(len(g) - win):
        a, b = g[i].msg, g[i + win].msg
        (xa, ya), (xb, yb) = _local_xy(a, lat0), _local_xy(b, lat0)
        ix = sum(g[j].msg.vx for j in range(i, i + win)) / 1000
        iy = sum(g[j].msg.vy for j in range(i, i + win)) / 1000
        worst = max(worst, math.hypot(xb - xa - ix, yb - ya - iy))

    return worst


def mean_vx(rx, t0, t1):
    g = of(window(rx, t0, t1), "GLOBAL_POSITION_INT")
    return sum(r.msg.vx for r in g) / len(g) / 100


def passive_flags(rx):
    return {
        "seq": seq_anomalies(rx) > 0,
        "time_back": time_regress(rx) > 0,
        "ts_frozen": ts_frozen(rx),
    }
