
import math
import random

R_EARTH = 6_378_137.0


class Fault:
    """Base class for time-bounded faults."""

    defaults = {}

    def __init__(self, idx, seed, start_s, duration_s=0.0, **params):
        unknown = set(params) - set(self.defaults)
        if unknown:
            raise ValueError(
                f"{type(self).__name__}: unknown parameters {sorted(unknown)}"
            )

        self.p = {**self.defaults, **params}
        self.start_us = round(start_s * 1e6)
        self.end_us = (
            self.start_us + round(duration_s * 1e6)
            if duration_s > 0
            else None
        )
        self.active = False
        self.rng = random.Random(seed * 1000 + idx)

    def match(self, name):
        """Return whether this fault applies to the message."""
        msgs = self.p.get("msgs")
        return not msgs or name in msgs

    def mutate(self, sim, name, fields, t):
        """Modify message fields before packing."""
        return fields

    def link(self, sim, name, deliveries):
        """Modify packed frames before delivery."""
        return deliveries

    def on_setpoint(self, sim, sp, t):
        """Modify a received setpoint."""
        return sp

    def on_start(self, sim, t):
        """Handle fault activation."""
        pass

    def on_end(self, sim, t):
        """Handle fault deactivation."""
        pass


class LinkLoss(Fault):
    """Drop or corrupt frames after MAVLink sequence assignment."""

    defaults = {
        "loss_prob": 0.1,
        "burst_len": 1,
        "corrupt": False,
        "msgs": [],
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._burst = 0

    def link(self, sim, name, deliveries):
        if not self.match(name):
            return deliveries

        if self._burst:
            self._burst -= 1
            hit = True
        elif self.rng.random() < self.p["loss_prob"]:
            self._burst = int(self.p["burst_len"]) - 1
            hit = True
        else:
            hit = False

        if not hit:
            return deliveries

        if not self.p["corrupt"]:
            return []

        out = []
        for delay_us, data in deliveries:
            frame = bytearray(data)

            index = (
                self.rng.randrange(10, len(frame) - 2)
                if len(frame) > 13
                else len(frame) - 1
            )
            frame[index] ^= 1 << self.rng.randrange(8)

            out.append((delay_us, bytes(frame)))

        return out


class LinkDelay(Fault):
    """Add delivery delay and jitter without changing timestamps."""

    defaults = {
        "delay_ms": 100.0,
        "jitter_ms": 0.0,
        "msgs": [],
    }

    def link(self, sim, name, deliveries):
        if not self.match(name):
            return deliveries

        out = []
        for delay_us, data in deliveries:
            jitter_ms = (
                self.rng.uniform(-1, 1) * self.p["jitter_ms"]
                if self.p["jitter_ms"]
                else 0
            )
            extra_us = max(
                0,
                int((self.p["delay_ms"] + jitter_ms) * 1000),
            )
            out.append((delay_us + extra_us, data))

        return out


class FcReboot(Fault):
    """Simulate a flight-controller reboot during flight."""

    defaults = {}

    def on_start(self, sim, t):
        sim.reboot_begin(t)

    def on_end(self, sim, t):
        sim.reboot_end(t)


class StaleData(Fault):
    """Freeze selected telemetry values while messages keep flowing."""

    defaults = {
        "msgs": ["ATTITUDE", "GLOBAL_POSITION_INT"],
        "fresh_timestamps": True,
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._frozen = {}

    def mutate(self, sim, name, fields, t):
        if not self.match(name):
            return fields

        if name not in self._frozen:
            self._frozen[name] = dict(fields)
            return fields

        out = dict(self._frozen[name])

        if self.p["fresh_timestamps"]:
            out["time_boot_ms"] = fields["time_boot_ms"]

        return out

    def on_end(self, sim, t):
        self._frozen.clear()


class GnssDrift(Fault):
    """Simulate gradual GNSS position drift."""

    defaults = {
        "rate_mps": 0.5,
        "bearing_deg": 90.0,
        "max_m": 30.0,
        "consistent_velocity": False,
    }

    def mutate(self, sim, name, fields, t):
        if name != "GLOBAL_POSITION_INT":
            return fields

        elapsed_s = (t - self.start_us) / 1e6
        offset_m = min(
            self.p["rate_mps"] * elapsed_s,
            self.p["max_m"],
        )
        bearing = math.radians(self.p["bearing_deg"])

        north_m = offset_m * math.cos(bearing)
        east_m = offset_m * math.sin(bearing)

        out = dict(fields)
        origin_lat = math.radians(sim.cfg["vehicle"]["origin_lat"])

        out["lat"] += round(
            math.degrees(north_m / R_EARTH) * 1e7
        )
        out["lon"] += round(
            math.degrees(
                east_m / (R_EARTH * math.cos(origin_lat))
            ) * 1e7
        )

        if (
            self.p["consistent_velocity"]
            and offset_m < self.p["max_m"]
        ):
            out["vx"] += round(
                self.p["rate_mps"] * math.cos(bearing) * 100
            )
            out["vy"] += round(
                self.p["rate_mps"] * math.sin(bearing) * 100
            )

        return out


class SetpointClamp(Fault):
    """Limit commanded speed without rejecting setpoints."""

    defaults = {"max_speed_mps": 3.0}

    def on_setpoint(self, sim, sp, t):
        out = dict(sp)
        limit = self.p["max_speed_mps"]
        current_limit = out.get("speed_limit")

        out["speed_limit"] = (
            limit
            if current_limit is None
            else min(limit, current_limit)
        )
        return out


FAULT_TYPES = {
    "link_loss": LinkLoss,
    "link_delay": LinkDelay,
    "fc_reboot": FcReboot,
    "stale_data": StaleData,
    "gnss_drift": GnssDrift,
    "setpoint_clamp": SetpointClamp,
}


def make_faults(cfg):
    """Build configured faults with reproducible random seeds."""
    seed = cfg["run"]["seed"]
    faults = []

    for idx, raw in enumerate(cfg["fault"]):
        params = dict(raw)
        fault_type = params.pop("type")

        try:
            cls = FAULT_TYPES[fault_type]
        except KeyError:
            raise ValueError(
                f"unknown fault type: {fault_type}"
            ) from None

        faults.append(cls(idx, seed, **params))

    return faults

