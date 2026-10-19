import copy

from helpers import base_cfg
from fcsim.sim import Sim

ALL = [
    {"type": "link_loss", "start_s": 10, "duration_s": 5, "loss_prob": 0.2, "burst_len": 2},
    {"type": "link_delay", "start_s": 12, "duration_s": 5, "delay_ms": 80, "jitter_ms": 30},
    {"type": "stale_data", "start_s": 20, "duration_s": 3},
    {"type": "gnss_drift", "start_s": 22, "duration_s": 8, "rate_mps": 0.4},
    {"type": "setpoint_clamp", "start_s": 25, "duration_s": 5, "max_speed_mps": 1.0},
    {"type": "fc_reboot", "start_s": 32, "duration_s": 4},
]


def run_sim(cfg, chunks=None):
    sim = Sim(copy.deepcopy(cfg))

    if chunks is None:
        sim.run()
    else:
        for t in chunks:
            sim.run(t)

    return list(sim.transport.frames)


def test_same_config_same_stream():
    cfg = base_cfg(ALL, duration=45.0, seed=5)
    assert run_sim(cfg) == run_sim(cfg)


def test_seed_changes_output():
    cfg1 = base_cfg(ALL, duration=45.0, seed=5)
    cfg2 = base_cfg(ALL, duration=45.0, seed=6)
    assert run_sim(cfg1) != run_sim(cfg2)


def test_chunked_run_matches_single_run():
    cfg = base_cfg(ALL, duration=45.0, seed=5)
    chunks = (7_000_000, 19_999_999, 31_000_000, 45_000_000)

    assert run_sim(cfg, chunks) == run_sim(cfg)