import heapq
import math
import random

from pymavlink.dialects.v20 import common as mav2

from .clock import VirtualClock
from .dynamics import Battery, Vehicle
from .faults import R_EARTH, make_faults
from .transport import RecordingTransport

# Flight modes used in HEARTBEAT.custom_mode.
NOT_READY, READY, MANUAL, EXTERNAL, HOLD = range(5)
MODE_NAMES = ["NOT_READY", "READY", "MANUAL", "EXTERNAL", "HOLD"]

# Telemetry schedule: (period_us, phase_us).
# Phases spread messages across time instead of emitting them in a burst.
TELEMETRY = {
    "HEARTBEAT": (1_000_000, 0),
    "ATTITUDE": (20_000, 5_000),
    "GLOBAL_POSITION_INT": (100_000, 10_000),
    "BATTERY_STATUS": (500_000, 15_000),
}
PHYS_PERIOD_US = 20_000

SEV_INFO, SEV_WARNING, SEV_ERROR = 6, 4, 3
CMD_DO_SET_MODE, CMD_ARM_DISARM = 176, 400
RES_OK, RES_DENIED, RES_UNSUPPORTED = 0, 2, 3
MASK_POS, MASK_VEL = 0b000111, 0b111000


class Sim:
    def __init__(self, cfg, clock=None, transport=None, record_tx=False):
        self.cfg = cfg
        self.clock = clock or VirtualClock()
        self.transport = transport or RecordingTransport()
        self.record_tx = record_tx

        self.rng = random.Random(cfg["run"]["seed"])
        self.vehicle = Vehicle(cfg["vehicle"])
        self.battery = Battery(cfg["battery"])

        fc = cfg["fc"]
        self.sys_id = fc["sys_id"]
        self.timeout_us = int(fc["setpoint_timeout_ms"] * 1000)

        self.mav_out = mav2.MAVLink(
            None, srcSystem=fc["sys_id"], srcComponent=fc["comp_id"]
        )
        self.mav_in = mav2.MAVLink(
            None, srcSystem=fc["sys_id"], srcComponent=fc["comp_id"]
        )
        self.mav_in.robust_parsing = True

        # Companion-computer identity used by scenario scripts.
        self.mav_cmp = mav2.MAVLink(None, srcSystem=1, srcComponent=191)
        self.cmp_seq = 0
        self.tx_seq = 0

        # Events are ordered by time, then insertion order.
        self.heap = []
        self._n = 0

        self.log = []       # (time_us, event_type, details)
        self.emitted = {}   # Messages generated before link faults.
        self.tx_dev = []    # (message_name, timing_deviation_us)

        # Flight-controller state.
        self.boot_us = 0
        self.down = False
        self.armed = False
        self.mode = NOT_READY
        self.sp = None
        self.sp_rx_us = None
        self._wd_pending = False
        self._low_batt_warned = False
        self._started = False

        self.faults = make_faults(cfg)
        self._bootstrap()

    # ------------------------------------------------------------------
    # Event loop

    def schedule(self, t, fn, *args):
        self._n += 1
        heapq.heappush(self.heap, (int(t), self._n, fn, args))

    def _bootstrap(self):
        for fault in self.faults:
            self.schedule(fault.start_us, self._fault_start, fault)
            if fault.end_us is not None:
                self.schedule(fault.end_us, self._fault_end, fault)

        for name, (_, phase) in TELEMETRY.items():
            self.schedule(phase, self._tele, name)

        self.schedule(0, self._physics)
        ready_us = int(self.cfg["fc"]["ready_after_s"] * 1e6)
        self.schedule(ready_us, self._ready_check)

        for script in self.cfg["script"]:
            self._schedule_script(dict(script))

    def run(self, until_us=None):
        """Run events until the requested virtual time."""
        until = int(
            until_us
            if until_us is not None
            else self.cfg["run"]["duration_s"] * 1e6
        )

        realtime = self.clock.realtime

        if realtime and not self._started:
            self.clock.reset()

        self._started = True

        while self.heap and self.heap[0][0] < until:
            t = self.heap[0][0]
            inbound = self.clock.wait_until(t, self.transport)

            # Incoming data may schedule an earlier event.
            if inbound:
                for t_rx, data in inbound:
                    self._on_bytes(t_rx, data)
                continue

            _, _, callback, args = heapq.heappop(self.heap)
            callback(t, *args)

        if not realtime:
            self.clock.wait_until(until, self.transport)

    # ------------------------------------------------------------------
    # Flight modes and setpoint watchdog

    def _set_mode(self, t, new, reason):
        if new == self.mode:
            return

        self._log(
            t,
            "mode",
            frm=MODE_NAMES[self.mode],
            to=MODE_NAMES[new],
            reason=reason,
        )
        self.mode = new

    def _ready_check(self, t):
        if self.down or self.mode != NOT_READY:
            return

        self._set_mode(t, READY, "preflight checks passed")
        self.statustext(t, SEV_INFO, "Ready for flight")

    def _stream_alive(self, t):
        return (
            self.sp_rx_us is not None
            and t - self.sp_rx_us <= self.timeout_us
        )

    def _arm_watchdog(self, t):
        if self._wd_pending:
            return

        self._wd_pending = True
        self.schedule(
            self.sp_rx_us + self.timeout_us + 1,
            self._watchdog,
        )

    def _watchdog(self, t):
        self._wd_pending = False

        if self.down or self.mode != EXTERNAL:
            return

        if t - self.sp_rx_us > self.timeout_us:
            self._set_mode(t, HOLD, "setpoint timeout")
            self.statustext(
                t,
                SEV_WARNING,
                f"Offboard setpoints lost >{self.timeout_us // 1000}ms: HOLD",
            )
        else:
            self._arm_watchdog(t)

    # ------------------------------------------------------------------
    # Scenario scripts (simulated companion computer) and MAVLink input

    def _cmp_frame(self, message):
        """Pack a simulated companion-computer message."""
        # Assign the next MAVLink sequence number.
        self.mav_cmp.seq = self.cmp_seq
        self.cmp_seq = (self.cmp_seq + 1) & 0xFF
        return message.pack(self.mav_cmp)

    def _schedule_script(self, script):
        """Schedule a scenario action at its configured start time."""
        t0 = int(script.pop("t_s") * 1e6)
        action = script.pop("do")

        if action in ("arm", "disarm"):
            # Send an arm or disarm command at the scheduled time.
            message = mav2.MAVLink_command_long_message(
                self.sys_id,
                1,
                CMD_ARM_DISARM,
                0,
                1.0 if action == "arm" else 0.0,
                0, 0, 0, 0, 0, 0,
            )
            self.schedule(
                t0,
                lambda t, m=message: self._on_bytes(t, self._cmp_frame(m)),
            )

        elif action == "mode":
            # Convert the configured mode name to its custom mode number.
            custom = MODE_NAMES.index(script["mode"])
            message = mav2.MAVLink_command_long_message(
                self.sys_id,
                1,
                CMD_DO_SET_MODE,
                0,
                1.0,
                float(custom),
                0, 0, 0, 0, 0,
            )
            self.schedule(
                t0,
                lambda t, m=message: self._on_bytes(t, self._cmp_frame(m)),
            )

        elif action == "setpoint_stream":
            # Schedule a stream of setpoints until the configured end time.
            until = int(script["until_s"] * 1e6)
            period = int(round(1e6 / script.get("rate_hz", 20.0)))
            self.schedule(
                t0,
                self._script_sp,
                script,
                period,
                until,
            )

        else:
            raise ValueError(f"Unknown scenario action: {action}")

    def _script_sp(self, t, script, period, until):
        """Send one scripted setpoint and schedule the next one."""
        if t >= until:
            return

        # Keep sending setpoints at the configured rate.
        self.schedule(t + period, self._script_sp, script, period, until)

        if script.get("kind", "vel") == "vel":
            # Ignore position and acceleration fields; command velocity.
            mask = 0x0DC7
            pos = (0, 0, 0)
            vel = (
                script.get("vx", 0.0),
                script.get("vy", 0.0),
                script.get("vz", 0.0),
            )
        else:
            # Ignore velocity and acceleration fields; command position.
            mask = 0x0DF8
            pos = (
                script.get("x", 0.0),
                script.get("y", 0.0),
                script.get("z", 0.0),
            )
            vel = (0, 0, 0)

        # Build a local NED position/velocity target message.
        message = mav2.MAVLink_set_position_target_local_ned_message(
            (t // 1000) & 0xFFFFFFFF,
            self.sys_id,
            1,
            1,
            mask,
            *pos,
            *vel,
            0, 0, 0, 0, 0,
        )
        self._on_bytes(t, self._cmp_frame(message))

    def _on_bytes(self, t, data):
        """Parse incoming MAVLink bytes and dispatch supported messages."""
        # Ignore incoming data while the flight controller is down.
        if self.down:
            return

        for message in self.mav_in.parse_buffer(data) or []:
            message_type = message.get_type()

            if message_type == "SET_POSITION_TARGET_LOCAL_NED":
                self._on_setpoint(t, message)
            elif message_type == "COMMAND_LONG":
                self._on_command(t, message)

    def _on_setpoint(self, t, message):
        """Convert an incoming MAVLink target into an internal setpoint."""
        if message.type_mask & MASK_POS == 0:
            # Position fields are enabled.
            setpoint = {
                "kind": "pos",
                "vec": [message.x, message.y, message.z],
                "speed_limit": None,
            }
        elif message.type_mask & MASK_VEL == 0:
            # Velocity fields are enabled.
            setpoint = {
                "kind": "vel",
                "vec": [message.vx, message.vy, message.vz],
                "speed_limit": None,
            }
        else:
            # Reject targets that specify neither supported control type.
            self._log(
                t,
                "setpoint_rejected",
                type_mask=message.type_mask,
            )
            return

        # Track reception time for the setpoint watchdog.
        self.sp_rx_us = t

        # Apply active faults before storing the setpoint.
        for fault in self.faults:
            if fault.active:
                setpoint = fault.on_setpoint(self, setpoint, t)

        self.sp = setpoint

    def _ack(self, t, command, result):
        self.emit(
            "COMMAND_ACK",
            dict(
                command=command,
                result=result,
                progress=0,
                result_param2=0,
                target_system=1,
                target_component=191,
            ),
            t,
        )

    def _on_command(self, t, message):
        if message.target_system not in (0, self.sys_id):
            return

        self._log(
            t,
            "cmd",
            command=message.command,
            p1=message.param1,
            p2=message.param2,
        )

        if message.command == CMD_ARM_DISARM:
            result = self._cmd_arm(t, message.param1 >= 0.5)
        elif message.command == CMD_DO_SET_MODE:
            result = self._cmd_mode(t, int(message.param2))
        else:
            result = RES_UNSUPPORTED

        self._ack(t, message.command, result)

    def _cmd_arm(self, t, arm):
        if arm:
            if self.armed:
                return RES_OK

            if self.mode != READY:
                self.statustext(t, SEV_ERROR, "Arm denied: not ready")
                return RES_DENIED

            self.armed = True
            self._set_mode(t, MANUAL, "armed")
            return RES_OK

        if self.armed:
            self.armed = False
            self._set_mode(t, READY, "disarmed")

        return RES_OK

    def _cmd_mode(self, t, custom):
        if custom not in (MANUAL, EXTERNAL, HOLD):
            return RES_DENIED

        if not self.armed:
            self.statustext(t, SEV_ERROR, "Mode change denied: disarmed")
            return RES_DENIED

        if custom == EXTERNAL:
            if not self._stream_alive(t):
                self.statustext(
                    t,
                    SEV_ERROR,
                    "EXTERNAL denied: no setpoint stream",
                )
                return RES_DENIED

            self._set_mode(t, EXTERNAL, "command")
            self._arm_watchdog(t)
            return RES_OK

        self._set_mode(t, custom, "command")
        return RES_OK

    # ------------------------------------------------------------------
    # MAVLink output

    def _log(self, t, kind, **details):
        """Record a simulation event with its timestamp and details."""
        self.log.append((t, kind, details))

    def emit(self, name, fields, t):
        """Apply message faults, pack the MAVLink frame, then apply link faults."""
        # Do not transmit messages while the flight controller is down.
        if self.down:
            return

        self.emitted[name] = self.emitted.get(name, 0) + 1

        # Apply active faults to message fields before packing.
        for fault in self.faults:
            if fault.active:
                fields = fault.mutate(self, name, fields, t)

        # Build and pack the MAVLink message using the current sequence number.
        message = getattr(
            mav2, f"MAVLink_{name.lower()}_message"
        )(**fields)

        self.mav_out.seq = self.tx_seq
        frame = message.pack(self.mav_out)
        self.tx_seq = (self.tx_seq + 1) & 0xFF

        # Apply active link faults to the packed frame and its delivery time.
        deliveries = [(0, frame)]
        for fault in self.faults:
            if fault.active:
                deliveries = fault.link(self, name, deliveries)

        # Send immediately or schedule delivery after the requested delay.
        for delay, data in deliveries:
            if delay <= 0:
                self._send(t, name, data)
            else:
                self.schedule(t + delay, self._send, name, data)

    def _send(self, t_sched, name, data):
        """Send a frame and record timing."""
        now = self.clock.now_us()
        self.transport.send(now, data)

        # Optionally record the delay relative to the scheduled send time.
        if self.record_tx:
            self.tx_dev.append((name, now - t_sched))

    def statustext(self, t, severity, text):
        """Log and transmit a MAVLink status message."""
        self._log(t, "text", severity=severity, text=text)

        self.emit(
            "STATUSTEXT",
            dict(
                severity=severity,
                text=text.encode()[:50],
                id=0,
                chunk_seq=0,
            ),
            t,
        )

    # ------------------------------------------------------------------
    # Telemetry

    def _tele(self, t, name):
        """Schedule and emit the next telemetry message."""
        period, _ = TELEMETRY[name]
        self.schedule(t + period, self._tele, name)

        if not self.down:
            fields = getattr(self, f"_f_{name}")(t)
            self.emit(name, fields, t)

    def _tb(self, t):
        """Return time since boot in milliseconds."""
        return ((t - self.boot_us) // 1000) & 0xFFFFFFFF

    def _f_HEARTBEAT(self, t):
        """Build the HEARTBEAT message."""
        base_mode = 1 | (128 if self.armed else 0)
        status = {NOT_READY: 1, READY: 3}.get(self.mode, 4)

        return dict(
            type=2,
            autopilot=12,
            base_mode=base_mode,
            custom_mode=self.mode,
            system_status=status,
            mavlink_version=3,
        )

    def _f_ATTITUDE(self, t):
        """Build the ATTITUDE message with simulated sensor noise."""
        vehicle = self.vehicle
        noise = self.cfg["vehicle"]["noise_att_rad"]
        gauss = self.rng.gauss

        return dict(
            time_boot_ms=self._tb(t),
            roll=vehicle.roll + gauss(0, noise),
            pitch=vehicle.pitch + gauss(0, noise),
            yaw=vehicle.yaw + gauss(0, noise),
            rollspeed=vehicle.rates[0],
            pitchspeed=vehicle.rates[1],
            yawspeed=vehicle.rates[2],
        )

    def _f_GLOBAL_POSITION_INT(self, t):
        """Build the global position message with simulated sensor noise."""
        cfg = self.cfg["vehicle"]
        vehicle = self.vehicle
        gauss = self.rng.gauss
        pos_noise = cfg["noise_pos_m"]

        # Add noise to local north, east, and down coordinates.
        north, east, down = (
            vehicle.p[0] + gauss(0, pos_noise),
            vehicle.p[1] + gauss(0, pos_noise),
            vehicle.p[2] + gauss(0, pos_noise),
        )

        # Convert local offsets to latitude and longitude.
        lat0 = math.radians(cfg["origin_lat"])
        lat = cfg["origin_lat"] + math.degrees(north / R_EARTH)
        lon = cfg["origin_lon"] + math.degrees(
            east / (R_EARTH * math.cos(lat0))
        )

        vel_noise = cfg["noise_vel_mps"]

        return dict(
            time_boot_ms=self._tb(t),
            lat=round(lat * 1e7),
            lon=round(lon * 1e7),
            alt=round((cfg["origin_alt_m"] - down) * 1000),
            relative_alt=round(-down * 1000),
            vx=round((vehicle.v[0] + gauss(0, vel_noise)) * 100),
            vy=round((vehicle.v[1] + gauss(0, vel_noise)) * 100),
            vz=round((vehicle.v[2] + gauss(0, vel_noise)) * 100),
            hdg=int(math.degrees(vehicle.yaw) % 360 * 100),
        )

    def _f_BATTERY_STATUS(self, t):
        """Build the BATTERY_STATUS message."""
        battery = self.battery
        cells = battery.c["cells"]

        # Pad unused cell-voltage entries with MAVLink's unknown value.
        voltages = [round(battery.cell_v * 1000)] * cells
        voltages += [0xFFFF] * (10 - cells)

        return dict(
            id=0,
            battery_function=0,
            type=1,
            temperature=0x7FFF,
            voltages=voltages,
            current_battery=round(battery.current_a * 100),
            current_consumed=round(battery.consumed_mah),
            energy_consumed=-1,
            battery_remaining=battery.remaining_pct,
        )

    # ------------------------------------------------------------------
    # Vehicle dynamics

    def _v_cmd(self):
        """Calculate the velocity command."""
        cfg = self.cfg["vehicle"]

        if not (self.armed and self.mode == EXTERNAL and self.sp):
            return [0.0, 0.0, 0.0]

        setpoint = self.sp

        if setpoint["kind"] == "vel":
            velocity = list(setpoint["vec"])
        else:
            # Apply proportional position control.
            velocity = [
                cfg["kp_pos"] * (setpoint["vec"][i] - self.vehicle.p[i])
                for i in range(3)
            ]

        # Limit the commanded speed.
        requested_limit = setpoint.get("speed_limit")
        limit = cfg["vmax_mps"]
        if requested_limit is not None:
            limit = min(limit, requested_limit)

        speed = math.sqrt(sum(v * v for v in velocity))
        if speed > limit:
            velocity = [v * limit / speed for v in velocity]

        return velocity

    def _physics(self, t):
        """Update vehicle and battery state."""
        self.schedule(t + PHYS_PERIOD_US, self._physics)

        if self.down:
            return

        dt = PHYS_PERIOD_US / 1e6
        self.vehicle.step(dt, self._v_cmd(), self.armed)

        speed = math.sqrt(sum(v * v for v in self.vehicle.v))
        self.battery.step(dt, speed, self.armed)

        threshold = self.cfg["battery"]["low_warn_pct"]
        if not self._low_batt_warned and self.battery.remaining_pct <= threshold:
            self._low_batt_warned = True
            self.statustext(
                t, SEV_WARNING, f"Low battery: {self.battery.remaining_pct}%"
            )

    # ------------------------------------------------------------------
    # Faults and reboot

    def _fault_start(self, t, fault):
        fault.active = True
        self._log(t, "fault_start", type=type(fault).__name__)
        fault.on_start(self, t)

    def _fault_end(self, t, fault):
        self._log(t, "fault_end", type=type(fault).__name__)
        fault.on_end(self, t)
        fault.active = False

    def reboot_begin(self, t):
        self.down = True
        self.armed = False

    def reboot_end(self, t):
        self.down = False
        self.boot_us = t
        self.tx_seq = 0
        self.sp = self.sp_rx_us = None
        self._wd_pending = False

        # The vehicle is frozen during reboot; reset its velocity on restart.
        self.vehicle.v = [0.0, 0.0, 0.0]
        self.mode = NOT_READY

        self._log(
            t,
            "reboot",
            note="FC restarted: time_boot_ms and seq reset, disarmed",
        )

        ready_us = int(self.cfg["fc"]["ready_after_s"] * 1e6)
        self.schedule(t + ready_us, self._ready_check)
        self.statustext(t, SEV_INFO, "Boot")
