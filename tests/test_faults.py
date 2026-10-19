import math

from helpers import (
    base_cfg, mean_vx, of, passive_flags, pos_vel_residual,
    repeated_values, run, seq_anomalies, time_regress,
    ts_frozen, window,
)

F0, F1 = 20.0, 35.0
R = 6378137.0


def fault(**kw):
    return {"start_s": F0, "duration_s": F1 - F0, **kw}


def simulate(*faults, duration=50.0):
    return run(base_cfg(list(faults), duration=duration))


def modes(sim, target):
    return [
        d for t, k, d in sim.log
        if k == "mode" and d["to"] == target
    ]


def test_link_loss_as_seq_gaps_and_rate_drop():
    _, rx, _ = simulate(fault(
        type="link_loss", loss_prob=0.1, burst_len=3
    ))

    assert seq_anomalies(window(rx, 0, F0)) == 0
    assert seq_anomalies(window(rx, F1 + 1, 50)) == 0
    assert seq_anomalies(window(rx, F0, F1)) > 20

    n = len(of(window(rx, F0, F1), "ATTITUDE"))
    assert 0.6 * 750 < n < 0.9 * 750
    assert len(of(window(rx, F0, F1), "HEARTBEAT")) >= 5


def test_link_loss_by_crc():
    _, rx, bad = simulate(fault(
        type="link_loss", loss_prob=0.2, corrupt=True
    ))
    assert bad > 50
    assert seq_anomalies(window(rx, F0, F1)) > 20


def test_constant_delay():
    _, rx, _ = simulate({
        "type": "link_delay", "start_s": 0.0, "delay_ms": 150.0
    })
    w = window(rx, 5, 45)

    lag = [r.t_us / 1000 - r.msg.time_boot_ms
           for r in of(w, "ATTITUDE")]
    assert all(abs(x - 150) < 1 for x in lag)
    assert not any(passive_flags(rx).values())

    ref = run(base_cfg())[1]
    assert len(of(w, "ATTITUDE")) == len(
        of(window(ref, 5, 45), "ATTITUDE")
    )


def test_delay_onset_and_offset_are_visible_only_at_the_transitions():
    _, rx, _ = simulate(fault(type="link_delay", delay_ms=150.0))

    assert passive_flags(rx)["time_back"]

    attitude = of(rx, "ATTITUDE")
    gap = max(b.t_us - a.t_us
              for a, b in zip(attitude, attitude[1:]))
    assert 150_000 < gap < 175_000

    assert not any(passive_flags(window(rx, F0 + 1, F1 - 1)).values())


def test_delay_jitter_causes_reordering_which_is_visible():
    _, rx, _ = simulate(fault(type="link_delay", delay_ms=150.0, jitter_ms=60.0
    ))
    assert time_regress(rx) > 5
    assert seq_anomalies(rx) > 5


def test_fc_reboot():
    sim, rx, _ = simulate(fault(type="fc_reboot", duration_s=8.0))

    assert not window(rx, F0 + 0.001, F0 + 8.0 - 0.001)

    after = [r for r in rx if r.t_us >= (F0 + 8.0) * 1e6]
    first = after[0]
    assert (first.name, first.msg.text, first.seq) == ("STATUSTEXT", "Boot", 0)

    assert of(after, "ATTITUDE")[0].msg.time_boot_ms < 100

    hb = of(after, "HEARTBEAT")[0].msg
    assert hb.custom_mode == 0
    assert not hb.base_mode & 128

    assert time_regress(rx) >= 1
    assert seq_anomalies(rx) >= 1
    assert not modes_after(sim, "EXTERNAL", F0 + 8)


def modes_after(sim, target, t_s):
    return [
        d for t, k, d in sim.log
        if k == "mode" and t > t_s * 1e6 and d["to"] == target
    ]


def test_stale_with_fresh_timestamps():
    _, rx, _ = simulate(
        fault(type="stale_data", fresh_timestamps=True), duration=45.0
    )

    assert not any(passive_flags(rx).values())

    hb = of(window(rx, F0, F1), "HEARTBEAT")
    assert len(hb) >= 14
    assert hb[0].msg.custom_mode == 3

    assert repeated_values(window(rx, F0, F1), "ATTITUDE")
    assert not repeated_values(window(rx, 8, F0), "ATTITUDE")

    pos = of(window(rx, F0 + 2, F1), "GLOBAL_POSITION_INT")
    assert len({(r.msg.lat, r.msg.lon) for r in pos}) == 1


def test_stale_with_frozen_timestamps():
    _, rx, _ = simulate(fault(
        type="stale_data", fresh_timestamps=False
    ))
    assert ts_frozen(rx)


def drift_err_m(rx_f, rx_0, t):
    a = of(window(rx_f, t, t + 1), "GLOBAL_POSITION_INT")[0].msg
    b = of(window(rx_0, t, t + 1), "GLOBAL_POSITION_INT")[0].msg

    dn = (a.lat - b.lat) / 1e7 * math.pi / 180 * R
    de = (
        (a.lon - b.lon) / 1e7 * math.pi / 180 * R
        * math.cos(math.radians(55.0))
    )
    return math.hypot(dn, de)


def test_gnss_drift_with_inconsistent_velocity():
    _, rx, _ = simulate(fault(
        type="gnss_drift", rate_mps=0.5, consistent_velocity=False
    ))
    ref = run(base_cfg())[1]

    assert drift_err_m(rx, ref, 34.0) > 6.0
    assert pos_vel_residual(window(rx, F0, F1 + 1), 55.0) > 1.0
    assert pos_vel_residual(window(ref, F0, F1 + 1), 55.0) < 0.3
    assert not any(passive_flags(rx).values())


def test_undetectable_gnss_drift():
    _, rx, _ = simulate(fault(
        type="gnss_drift", rate_mps=0.5, consistent_velocity=True
    ))
    ref = run(base_cfg())[1]

    assert drift_err_m(rx, ref, 34.0) > 6.0
    assert pos_vel_residual(window(rx, F0, F1 - 0.5), 55.0) < 0.3
    assert not any(passive_flags(rx).values())


def test_setpoint_clamp_is_silent():
    sim, rx, _ = simulate(fault(
        type="setpoint_clamp", max_speed_mps=2.0
    ))

    assert 4.7 < mean_vx(rx, 15, 19) < 5.3
    assert 1.8 < mean_vx(rx, 25, 34) < 2.2
    assert not modes(sim, "HOLD")

    hb = of(window(rx, F0, F1), "HEARTBEAT")
    assert all(r.msg.custom_mode == 3 for r in hb)

    assert not any(passive_flags(rx).values())
    assert abs(mean_vx(rx, 25, 34) - 5.0) > 1.0


def test_setpoint_clamp_zero():
    sim, rx, _ = simulate(fault(
        type="setpoint_clamp", max_speed_mps=0.0
    ))

    assert abs(mean_vx(rx, 28, 34)) < 0.1
    assert not modes(sim, "HOLD")
