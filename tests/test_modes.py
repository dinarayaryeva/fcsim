from helpers import base_cfg, decode, mean_vx, of, run
from fcsim import config
from fcsim.sim import Sim


def modes(sim):
    return [(t / 1e6, d["to"], d["reason"])
            for t, k, d in sim.log if k == "mode"]


def mk(script, duration=40.0):
    return config.build({
        "run": {"seed": 3, "duration_s": duration},
        "script": script,
    })


def arm(t=4.0):
    return {"t_s": t, "do": "arm"}


def stream(t, until, vx=2.0):
    return {
        "t_s": t, "do": "setpoint_stream", "kind": "vel",
        "vx": vx, "rate_hz": 20, "until_s": until,
    }


def mode(t=5.0):
    return {"t_s": t, "do": "mode", "mode": "EXTERNAL"}


def test_boot_ready_and_arm():
    sim = Sim(mk([arm(1.0), arm()], 6.0))
    sim.run()

    # Check boot readiness and arm state transitions.
    m = modes(sim)
    assert [(x[0], x[1]) for x in m] == [
        (3.0, "READY"), (4.0, "MANUAL")
    ]

    # The first arm request is denied; the second is accepted.
    rx, _ = decode(sim.transport.frames)
    assert [r.msg.result for r in of(rx, "COMMAND_ACK")] == [2, 0]
    assert any("Arm denied" in r.msg.text for r in of(rx, "STATUSTEXT"))


def test_external_requires_setpoints():
    sim = Sim(mk([arm(), mode()], 8.0))
    sim.run()

    # External mode must be denied without a setpoint stream.
    assert [x[1] for x in modes(sim)] == ["READY", "MANUAL"]
    rx, _ = decode(sim.transport.frames)
    assert any("EXTERNAL denied" in r.msg.text for r in of(rx, "STATUSTEXT"))


def test_setpoint_timeout():
    sim = Sim(mk([
        arm(), stream(4.5, 10.0), mode(),
    ], 15.0))
    sim.run()

    # Check that HOLD starts just after the 500 ms timeout.
    t, _, reason = next(
        (t, to, reason) for t, to, reason in modes(sim) if to == "HOLD"
    )
    gap = t * 1e6 - sim.sp_rx_us
    assert 500_000 < gap <= 501_000
    assert reason == "setpoint timeout"

    # Check the warning and the reported HOLD state.
    rx, _ = decode(sim.transport.frames)
    assert any("HOLD" in r.msg.text for r in of(rx, "STATUSTEXT"))
    hb = of(rx, "HEARTBEAT")[-1].msg
    assert hb.custom_mode == 4 and hb.base_mode & 128


def test_setpoint_gaps():
    sim = Sim(mk([
        arm(), stream(4.5, 8.0, 1.0), mode(),
        stream(8.3, 12.0, 1.0),
        stream(12.7, 16.0, 1.0),
    ], 20.0))
    sim.run()

    # The short gap is tolerated; the longer gap triggers HOLD once.
    holds = [t for t, to, _ in modes(sim) if to == "HOLD"]
    assert len(holds) == 1
    assert 12.4 < holds[0] < 12.5


def test_hold_recovers_by_command():
    sim = Sim(mk([
        arm(), stream(4.5, 8.0, 3.0), mode(),
        stream(12.0, 30.0, 3.0), mode(14.0),
    ], 30.0))
    sim.run()

    # Setpoints alone must not exit HOLD; a mode command is required.
    m = modes(sim)
    assert any(to == "HOLD" for _, to, _ in m)
    assert m[-1][1] == "EXTERNAL" and round(m[-1][0]) == 14


def test_velocity_and_hold_braking():
    sim, rx, _ = run(base_cfg(duration=20.0))

    # Check that velocity reaches the commanded 5 m/s.
    assert 4.7 < mean_vx(rx, 12, 18) < 5.3

    # Check that the vehicle moves north.
    pos = of(rx, "GLOBAL_POSITION_INT")
    assert pos[-1].msg.lat > pos[0].msg.lat

    # Check that the vehicle stops after entering HOLD.
    sim = Sim(mk([arm(), stream(4.5, 10.0, 5.0), mode()], 20.0))
    sim.run()
    rx, _ = decode(sim.transport.frames)
    assert abs(mean_vx(rx, 15, 20)) < 0.1


def test_position_setpoint():
    sim = Sim(mk([
        arm(),
        {
            "t_s": 4.5, "do": "setpoint_stream", "kind": "pos",
            "x": 20.0, "y": 0.0, "z": -10.0,
            "rate_hz": 20, "until_s": 30.0,
        },
        mode(),
    ], 30.0))
    sim.run()

    # Check that the vehicle converges to the target position.
    assert abs(sim.vehicle.p[0] - 20) < 0.5
    assert abs(sim.vehicle.p[2] + 10) < 0.5
