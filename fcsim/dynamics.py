import math

G = 9.80665  # Gravitational acceleration (m/s²)


class Vehicle:
    def __init__(self, cfg):
        self.c = cfg
        self.p = [0.0, 0.0, 0.0]  # Position: north, east, down (m)
        self.v = [0.0, 0.0, 0.0]  # Velocity: north, east, down (m/s)
        self.roll = self.pitch = self.yaw = 0.0  # Attitude (rad)
        self.rates = [0.0, 0.0, 0.0]  # Roll, pitch, and yaw rates (rad/s)

    def step(self, dt, v_cmd, armed):
        c = self.c

        if not armed:
            self.v = [0.0, 0.0, 0.0]
            self.rates = [0.0, 0.0, 0.0]
            self.roll = self.pitch = 0.0
            return

        # Smoothly move velocity toward the commanded value.
        k = 1.0 - math.exp(-dt / c["tau_s"])
        v_old = list(self.v)
        self.v = [
            v + (vc - v) * k
            for v, vc in zip(self.v, v_cmd)
        ]

        # Integrate position using the updated velocity.
        for i in range(3):
            self.p[i] += self.v[i] * dt

        # Prevent the vehicle from moving below ground level.
        if self.p[2] > 0.0:
            self.p[2] = 0.0
            self.v[2] = min(self.v[2], 0.0)

        # Estimate horizontal acceleration and corresponding attitude.
        an = (self.v[0] - v_old[0]) / dt
        ae = (self.v[1] - v_old[1]) / dt
        roll = math.atan2(ae, G)
        pitch = -math.atan2(an, G)  # Heading zero points north.

        # Estimate angular rates from attitude changes.
        self.rates = [
            (roll - self.roll) / dt,
            (pitch - self.pitch) / dt,
            0.0,
        ]
        self.roll, self.pitch = roll, pitch


class Battery:
    def __init__(self, cfg):
        self.c = cfg
        self.consumed_mah = 0.0
        self.current_a = cfg["i_idle_a"]

    def step(self, dt, speed, armed):
        c = self.c

        # Model current draw from flight state and speed.
        if armed:
            self.current_a = c["i_hover_a"] + c["i_per_mps2_a"] * speed**2
        else:
            self.current_a = c["i_idle_a"]

        # Convert amp-seconds to milliamp-hours.
        self.consumed_mah += self.current_a * dt / 3.6

    @property
    def remaining(self) -> float:
        """Return the remaining battery fraction (0.0-1.0)."""
        return max(
            0.0,
            1.0 - self.consumed_mah / self.c["capacity_mah"],
        )

    @property
    def remaining_pct(self) -> int:
        """Return the remaining battery charge as a percentage."""
        return int(round(self.remaining * 100))

    @property
    def cell_v(self) -> float:
        """Estimate per-cell voltage from charge and current draw."""
        v_oc = 3.3 + 0.9 * self.remaining
        voltage_drop = self.c["r_int_ohm"] * self.current_a / self.c["cells"]
        return v_oc - voltage_drop
