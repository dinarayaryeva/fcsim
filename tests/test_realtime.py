import socket
import threading
import time

from pymavlink.dialects.v20 import common as mav2

from fcsim.clock import RealClock
from fcsim.sim import Sim
from fcsim.transport import RecordingTransport, UdpTransport
from helpers import base_cfg


LIMIT_US = 1000


def test_jitter():
    sim = Sim(
        base_cfg(duration=6.0),
        RealClock(),
        RecordingTransport(),
        record_tx=True,
    )
    sim.run()

    # Group transmission timing deviations by message type.
    by = {}
    for name, deviation in sim.tx_dev:
        by.setdefault(name, []).append(deviation)

    all_deviations = []
    for name in (
        "HEARTBEAT",
        "ATTITUDE",
        "GLOBAL_POSITION_INT",
        "BATTERY_STATUS",
    ):
        deviations = sorted(by[name])
        all_deviations.extend(deviations)

        # Messages must not be sent before their deadlines.
        assert deviations[0] >= 0

        # Check the median timing deviation.
        median = deviations[len(deviations) // 2]
        assert median <= 500, (name, "median", median)

    # Check p99 across all streams.
    all_deviations.sort()
    p99 = all_deviations[int(len(all_deviations) * 0.99)]
    assert p99 <= LIMIT_US, ("p99", p99)

    # Verify that the 50 Hz attitude stream was not interrupted.
    assert len(by["ATTITUDE"]) >= 295


def test_udp_end_to_end():
    # Find an available local UDP port for the simulator.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    sim_port = probe.getsockname()[1]
    probe.close()

    # Create the client socket and set a receive timeout.
    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cli.bind(("127.0.0.1", 0))
    cli.settimeout(0.05)

    cfg = base_cfg(duration=2.5)
    cfg["fc"]["ready_after_s"] = 0.2
    cfg["script"] = []

    # Configure the simulator to send packets to the client.
    tr = UdpTransport(
        f"127.0.0.1:{sim_port}",
        f"127.0.0.1:{cli.getsockname()[1]}",
    )
    sim = Sim(cfg, RealClock(), tr, record_tx=True)

    # Run the simulator in a background thread.
    th = threading.Thread(target=sim.run)
    th.start()
    time.sleep(0.6)

    # Send a MAVLink arm command from the client.
    me = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    command = mav2.MAVLink_command_long_message(
        1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
    ).pack(me)
    cli.sendto(command, ("127.0.0.1", sim_port))

    # Collect incoming MAVLink messages for up to 1.5 seconds.
    parser = mav2.MAVLink(None)
    parser.robust_parsing = True
    received = []
    deadline = time.time() + 1.5

    while time.time() < deadline:
        try:
            received.extend(parser.parse_buffer(cli.recv(2048)) or [])
        except socket.timeout:
            pass

    th.join()

    # Check that telemetry and the command acknowledgement arrived.
    kinds = {msg.get_type() for msg in received}
    assert {
        "HEARTBEAT",
        "ATTITUDE",
        "GLOBAL_POSITION_INT",
        "COMMAND_ACK",
    } <= kinds

    # Verify that the arm command was accepted.
    acks = [
        msg.result
        for msg in received
        if msg.get_type() == "COMMAND_ACK"
    ]
    assert acks == [0]

    # Verify that the heartbeat reports the vehicle as armed.
    assert any(
        msg.get_type() == "HEARTBEAT" and msg.base_mode & 128
        for msg in received
    )
