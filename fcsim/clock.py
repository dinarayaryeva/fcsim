import time


class VirtualClock:
    """Fast, deterministic clock for tests and simulation."""

    realtime = False

    def __init__(self):
        self._now = 0  # Simulated time in microseconds

    def reset(self):
        self._now = 0

    def now_us(self) -> int:
        return self._now

    def wait_until(self, t_us, transport):
        # Advance time instantly without waiting for I/O.
        if t_us > self._now:
            self._now = t_us
        return None


class RealClock:
    """Real-time clock with precise event deadlines."""

    realtime = True
    SPIN_US = 1500  # Switch to active waiting near the deadline

    def __init__(self):
        self._t0 = time.perf_counter_ns()  # Monotonic start time

    def reset(self):
        self._t0 = time.perf_counter_ns()

    def now_us(self) -> int:
        # Elapsed time in microseconds since reset.
        return (time.perf_counter_ns() - self._t0) // 1000

    def wait_until(self, t_us, transport):
        """Wait for a deadline or return received packets."""
        while True:
            remaining = t_us - self.now_us()
            if remaining <= 0:
                return None

            # Sleep until near the deadline to reduce CPU usage.
            timeout = (
                (remaining - self.SPIN_US) / 1e6
                if remaining > self.SPIN_US
                else 0
            )

            data = transport.poll(max(timeout, 0))
            if data:
                # Timestamp packets when they are processed.
                now = self.now_us()
                return [(now, d) for d in data]
