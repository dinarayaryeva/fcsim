import argparse
import sys

from . import config
from .clock import RealClock
from .sim import Sim
from .transport import RecordingTransport, UdpTransport


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fcsim")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run scenario")
    r.add_argument("scenario")
    r.add_argument("--realtime", action="store_true", help="realtime + UDP (otherwise virtual time)")
    a = ap.parse_args(argv)

    cfg = config.load(a.scenario)

    if a.realtime:
        tr = UdpTransport(cfg["link"]["bind"], cfg["link"]["peer"])
        sim = Sim(cfg, RealClock(), tr, record_tx=True)
        print(f"realtime: listens to {cfg['link']['bind']}, sends to {cfg['link']['peer']}")
    else:
        tr = RecordingTransport()
        sim = Sim(cfg, transport=tr)
    try:
        sim.run()
    except KeyboardInterrupt:
        pass

    print(f"Generated msgs: {sim.emitted}")
    for t, kind, d in sim.log:
        if kind in ("mode", "fault_start", "fault_end", "reboot", "text"):
            print(f"  t={t / 1e6:8.3f}s  {kind:11s} {d}")
    if a.realtime:
        devs = [d for _, d in sim.tx_dev]
        if devs:
            devs.sort()
            print(f"delay from the schedule, us: median {devs[len(devs) // 2]}, "
                  f"p99 {devs[int(len(devs) * .99)]}, max {devs[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
