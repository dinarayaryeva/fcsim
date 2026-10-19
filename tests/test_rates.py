import pytest

from fcsim import config
from helpers import base_cfg, of, run, seq_anomalies


# Expected message periods and counts over 10 seconds.
EXPECT_10S = {
    "HEARTBEAT": (1.0, 10),
    "ATTITUDE": (0.02, 500),
    "GLOBAL_POSITION_INT": (0.1, 100),
    "BATTERY_STATUS": (0.5, 20),
}


@pytest.mark.parametrize("name", EXPECT_10S)
def test_rates(name):
    sim, rx, bad = run(base_cfg(duration=10.0))
    period, count = EXPECT_10S[name]
    ts = [r.t_us for r in of(rx, name)]

    # Check the message count and exact transmission period.
    assert len(ts) == count
    assert {b - a for a, b in zip(ts, ts[1:])} == {int(period * 1e6)}

    # Check that timestamps match the simulated boot time.
    if name in ("ATTITUDE", "GLOBAL_POSITION_INT"):
        assert all(r.msg.time_boot_ms == r.t_us // 1000 for r in of(rx, name))


def test_seq():
    sim, rx, bad = run(base_cfg(duration=10.0))

    # Ensure there are no bad packets or sequence anomalies.
    assert bad == 0 and seq_anomalies(rx) == 0


def test_statustext():
    sim, rx, _ = run(base_cfg(duration=10.0))
    texts = [r.msg.text for r in of(rx, "STATUSTEXT")]

    # Check the startup status message.
    assert texts == ["Ready for flight"]


def test_config_param():
    from fcsim.faults import make_faults

    # Reject unknown or invalid parameters.
    with pytest.raises(ValueError):
        config.build({"vehicle": {"tau": 1}})

    with pytest.raises(ValueError):
        make_faults(config.build({"fault": [{"type": "nope", "start_s": 1}]}))

    with pytest.raises(ValueError):
        make_faults(config.build({"fault": [{"type": "link_loss", "start_s": 1, "lossprob": 1}]}))
