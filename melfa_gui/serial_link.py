"""
Serial protocol wrapper for the Mitsubishi RV-2AJ / CR1-571 controller.

Non-procedural ASCII protocol: "1;1;<CMD>\\r" -> reply "Qo..." (OK) or
"Qe"/"Qx..." (error). A lowercase status char ("Qok", "Qer") means an
alarm is pending on the controller.

Verified on this controller: STATE, PPOSF, JPOSF, VAL<var>, CNTLON,
CNTLOFF, SRVON, SRVOFF, STOP (also halts an EXECMOV in progress),
EXECJOVRD, the relative joint move in send_step_move(), the TCP move
in send_tcp_move(), and the transfer-line I/O below. RUN is from
vendor forum threads and has not been tested here.

Continuous JOG does not exist over RS-232C on this controller generation
(teaching-pendant-only), so the UI uses discrete relative joint steps.
"""

import enum
import math
import threading
import time

import serial
import serial.tools.list_ports

PORT_DEFAULT = "/dev/ttyUSB0"
BAUDRATE = 9600
PARITY = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_TWO
BYTESIZE = serial.EIGHTBITS
READ_TIMEOUT = 2.0


# J4 is not fitted on the 5-axis RV-2AJ (JPOSF reports ****).
STEPPABLE_JOINTS = (1, 2, 3, 5, 6)
TCP_AXES = ("X", "Y", "Z")
RADIAL = "R"  # along the arm's reach, see send_tcp_move
MIN_RADIAL_RADIUS_MM = 50.0

# -- Festo MPS 500 transfer line ------------------------------------------
# The robot is the "Robot with assembly" station, wired to Transfersystem
# Station 4 over connector XPA4 (24-pin SysLink). Signal map from the
# system's communication plan. Robot outputs 0-3 are not connected to the
# line, and the belt itself is not robot-controlled.
#
# Serial forms verified on this controller (2026-09-25): EXECM_OUT(n)=0/1
# (needs CNTLON; QoK and read-back matched on outputs 0-15), VALM_OUT(n),
# VALM_IN(n), and the byte reads VALM_INB(0) / VALM_OUTB(0) ("QoKM_INB=+16").
LINE_IO_BYTE = 0  # every transfer-line signal is in bits 0-7

IN_ORDER_BIT0 = 6       # from Station 4 OUT2
IN_ORDER_BIT1 = 7       # from Station 4 OUT3

OUT_ERR_CODE_BIT0 = 4   # to Station 4 IN0
OUT_ERR_CODE_BIT1 = 5   # to Station 4 IN1
OUT_BUSY = 6            # to Station 4 IN2, "order active/busy"
OUT_STATION_READY = 7   # to Station 4 IN3

ERROR_CODE_MAX = 3


class Order(enum.Enum):
    NONE = "No order"
    ASSEMBLE = "Assemble"
    SORT_OUT = "Sorting out"
    TAKE_FROM_PALLET = "Take part on pallet"  # "at bus: take part on pallet"


# (Bit1 = IN7, Bit0 = IN6) -> order
ORDER_BY_BITS = {
    (0, 0): Order.NONE,
    (0, 1): Order.ASSEMBLE,
    (1, 0): Order.SORT_OUT,
    (1, 1): Order.TAKE_FROM_PALLET,
}


def bit(byte_value, n):
    """Bit n (0-7) of a byte read with read_inputs()/read_outputs()."""
    return (byte_value >> (n - 8 * LINE_IO_BYTE)) & 1


def order_bits(inputs):
    """(Bit1, Bit0) of the order request in an input byte."""
    return bit(inputs, IN_ORDER_BIT1), bit(inputs, IN_ORDER_BIT0)


def decode_order(inputs):
    """Order requested by the transfer line, from an input byte."""
    return ORDER_BY_BITS[order_bits(inputs)]


def error_code(outputs):
    """Error code (0-3) the robot is asserting, from an output byte."""
    return bit(outputs, OUT_ERR_CODE_BIT1) << 1 | bit(outputs, OUT_ERR_CODE_BIT0)


class SerialLinkError(Exception):
    pass


class CommandRejected(SerialLinkError):
    """Controller replied, but not with Qo. `reply` is the exact reply."""

    def __init__(self, message, reply=None):
        super().__init__(message)
        self.reply = reply


class SerialLink:
    """Thread-safe wrapper around the controller's serial protocol.

    All sends are serialized with a lock so the background poll loop and
    UI-triggered commands (including the emergency stop) never interleave
    bytes on the single physical port.
    """

    def __init__(self):
        self._ser = None
        self._lock = threading.Lock()

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def connect(self, port=PORT_DEFAULT):
        with self._lock:
            self._close_locked()
            try:
                self._ser = serial.Serial(
                    port=port,
                    baudrate=BAUDRATE,
                    bytesize=BYTESIZE,
                    parity=PARITY,
                    stopbits=STOPBITS,
                    timeout=READ_TIMEOUT,
                )
            except serial.SerialException as e:
                self._ser = None
                raise SerialLinkError(f"could not open {port}: {e}") from e

    def disconnect(self):
        with self._lock:
            self._close_locked()

    def _close_locked(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def send(self, command):
        """Send "1;1;<command>\\r" and return the raw reply string.

        Raises SerialLinkError if the port isn't open, on a serial
        exception (which also closes the port), or if no reply arrives
        within the timeout.
        """
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                raise SerialLinkError("port not open")
            try:
                self._ser.reset_input_buffer()
                self._ser.write(f"1;1;{command}\r".encode("ascii"))
                self._ser.flush()
                raw = self._read_reply_locked()
            except serial.SerialException as e:
                self._close_locked()
                raise SerialLinkError(f"serial error: {e}") from e

        if not raw:
            raise SerialLinkError(f"no reply to {command!r}")
        return raw

    def _read_reply_locked(self):
        deadline = time.time() + READ_TIMEOUT
        buf = bytearray()
        while time.time() < deadline:
            chunk = self._ser.read(1)
            if not chunk:
                break
            buf += chunk
            if buf.endswith(b"\r"):
                break
        return buf.decode("ascii", errors="replace")

    # -- confirmed read-only queries ---------------------------------

    def query_state(self):
        return self.send("STATE")

    def query_pposf(self):
        return self.send("PPOSF")

    def query_jposf(self):
        return self.send("JPOSF")

    # -- confirmed state-changing commands ----------------------------
    # These affect robot state. Callers must only invoke them from
    # explicit user action in the UI, never automatically on connect or
    # from the background poll loop.

    def send_cntlon(self):
        return self.send("CNTLON")

    def send_srvon(self):
        return self.send("SRVON")

    def send_srvoff(self):
        return self.send("SRVOFF")

    def send_run(self, program_number):
        return self.send(f"RUN {program_number}")

    def send_stop(self):
        """Program/motion stop. Safe to call at any time, any state."""
        return self.send("STOP")

    def send_ovrd(self, percent):
        percent = max(1, min(100, int(percent)))
        return self.send(f"EXECJOVRD {percent}")

    def send_cntloff(self):
        return self.send("CNTLOFF")

    def expect_ok(self, command):
        raw = self.send(command)
        if not is_ok_reply(raw):
            raise CommandRejected(f"{command!r} rejected: {raw.strip()}", reply=raw.strip())
        return raw

    # -- transfer line I/O ---------------------------------------------

    def _read_byte(self, variable):
        raw = self.expect_ok(f"VAL{variable}({LINE_IO_BYTE})")
        try:
            value = int(raw.strip().rsplit("=", 1)[1])
        except (IndexError, ValueError) as e:
            raise CommandRejected(f"unexpected VAL{variable} reply: {raw.strip()}", reply=raw.strip()) from e
        if not 0 <= value <= 0xFF:
            raise CommandRejected(f"unexpected VAL{variable} reply: {raw.strip()}", reply=raw.strip())
        return value

    def read_inputs(self):
        """Robot input bits 0-7 as an int (see IN_* and decode_order)."""
        return self._read_byte("M_INB")

    def read_outputs(self):
        """Robot output bits 0-7 as an int (see OUT_* and error_code)."""
        return self._read_byte("M_OUTB")

    def _set_outputs(self, values):
        """Write {output: 0/1} one bit at a time, then read back and check.

        Only the named bits are written (never M_OUTB, which would also
        overwrite the other outputs). Returns the output byte read back.
        """
        self.expect_ok("CNTLON")  # EXEC needs operation rights; no motion
        for n, v in values.items():
            self.expect_ok(f"EXECM_OUT({n})={1 if v else 0}")
        outputs = self.read_outputs()
        wrong = [n for n, v in values.items() if bit(outputs, n) != (1 if v else 0)]
        if wrong:
            raise CommandRejected(
                f"output read-back mismatch on OUT{wrong}: wanted {values}, "
                f"M_OUTB={outputs:08b}"
            )
        return outputs

    def set_station_ready(self, ready):
        return self._set_outputs({OUT_STATION_READY: ready})

    def set_busy(self, busy):
        return self._set_outputs({OUT_BUSY: busy})

    def set_error_code(self, code):
        """Assert error code 0-3 on OUT5 (bit 1) / OUT4 (bit 0).

        The two bits are written one after the other, so the line can see
        an in-between code for one serial round trip (~0.1 s). Bits going
        1->0 are cleared before bits going 0->1, so that in-between code is
        never higher than both the old and the new code.
        """
        if not isinstance(code, int) or not 0 <= code <= ERROR_CODE_MAX:
            raise ValueError(f"error code must be 0-{ERROR_CODE_MAX}, got {code!r}")
        want = {OUT_ERR_CODE_BIT0: code & 1, OUT_ERR_CODE_BIT1: code >> 1 & 1}
        current = self.read_outputs()
        order = sorted(want, key=lambda n: want[n])  # clears first, then sets
        changes = {n: want[n] for n in order if bit(current, n) != want[n]}
        if not changes:
            return current
        return self._set_outputs(changes)

    def send_step_move(self, joint_number, offset_degrees, override_percent):
        """Move one joint by offset_degrees relative to where it is now.

        Sequence verified on this controller (J1/J5, +/-2 and 5 deg):
        JTMP=J_CURR, add RAD(offset) to one component, read JTMP back with
        VAL to check only that component changed by the offset, then
        EXECMOV JTMP. `J_CURR+(tuple)` is NOT accepted (alarm L4220).

        EXECMOV replies as soon as motion starts; watch JPOSF to know when
        it has finished. Returns the target angle in degrees.
        """
        if joint_number not in STEPPABLE_JOINTS:
            raise ValueError(f"J{joint_number} cannot be stepped on this robot")
        n = joint_number
        self.expect_ok(f"EXECJOVRD {max(1, min(100, int(override_percent)))}")
        self.expect_ok("EXECJTMP=J_CURR")
        base = parse_val_tuple(self.expect_ok("VALJTMP"))
        self.expect_ok(f"EXECJTMP.J{n}=JTMP.J{n}+RAD({offset_degrees:.2f})")
        target = parse_val_tuple(self.expect_ok("VALJTMP"))

        expected = base[n - 1] + offset_degrees
        others_changed = any(
            abs(target[i] - base[i]) > 0.005 for i in range(len(base)) if i != n - 1
        )
        if abs(target[n - 1] - expected) > 0.05 or others_changed:
            raise CommandRejected(
                f"target check failed: base {base}, target {target}, "
                f"expected J{n}={expected:.2f}; not moving"
            )
        self.expect_ok("EXECMOV JTMP")
        return target[n - 1]

    def send_joint_move(self, joints, override_percent):
        """Absolute joint move to (J1..J6) in degrees; None (absent J4) -> unchanged.

        Built only from the syntax verified on this controller (send_step_move):
        JTMP=J_CURR, then per joint JTMP.Jn=JTMP.Jn+RAD(target-current). The
        absolute tuple form JTMP=(j1,...,j6) was never confirmed here (tuple
        expressions such as J_CURR+(...) raise L4220), so it is not used.
        JTMP is read back and every joint checked before EXECMOV.
        """
        if len(joints) != 6:
            raise ValueError("expected 6 joint values")
        self.expect_ok(f"EXECJOVRD {max(1, min(100, int(override_percent)))}")
        self.expect_ok("EXECJTMP=J_CURR")
        base = parse_val_tuple(self.expect_ok("VALJTMP"))
        if len(base) < 6:
            raise CommandRejected(f"unexpected VALJTMP reply: {base}; not moving")
        wanted = list(base[:6])
        for n in STEPPABLE_JOINTS:
            if joints[n - 1] is None:
                continue
            delta = round(float(joints[n - 1]) - base[n - 1], 2)
            wanted[n - 1] = base[n - 1] + delta
            if abs(delta) >= 0.01:
                self.expect_ok(f"EXECJTMP.J{n}=JTMP.J{n}+RAD({delta:.2f})")
        target = parse_val_tuple(self.expect_ok("VALJTMP"))
        if len(target) < 6 or any(abs(target[i] - wanted[i]) > 0.011 for i in range(6)):
            raise CommandRejected(f"target check failed: wanted {wanted}, controller has {target}; not moving")
        self.expect_ok("EXECMOV JTMP")
        return target[:6]

    def send_tcp_move(self, axis, distance_mm, override_percent):
        """Move the TCP by distance_mm, straight, relative to where it is now.

        axis "X"/"Y"/"Z" moves along that base-frame axis. axis "R" moves
        along the arm's reach: horizontally out from (positive) or in toward
        (negative) the base's vertical axis, in the direction the arm points.
        On this 5-axis arm an "R" move never needs J1 to rotate.

        Verified on hardware: EXECPTMP=P_CURR, component edits like
        PTMP.X=PTMP.X+5.00, VALPTMP read-back, EXECMOV PTMP. Every edited
        component is checked in the read-back before EXECMOV is sent.
        Returns the X/Y/Z target in mm.
        """
        if axis not in TCP_AXES and axis != RADIAL:
            raise ValueError(f"unknown TCP axis {axis!r}")
        self.expect_ok(f"EXECJOVRD {max(1, min(100, int(override_percent)))}")
        self.expect_ok("EXECPTMP=P_CURR")
        base = parse_val_tuple(self.expect_ok("VALPTMP"))
        if len(base) < 3:
            raise CommandRejected(f"unexpected VALPTMP reply: {base}; not moving")

        if axis == RADIAL:
            radius = math.hypot(base[0], base[1])
            if radius < MIN_RADIAL_RADIUS_MM:
                raise CommandRejected(
                    f"TCP is {radius:.0f} mm from the base axis; too close to define "
                    f"forward/back (needs {MIN_RADIAL_RADIUS_MM:.0f} mm); not moving"
                )
            deltas = {0: distance_mm * base[0] / radius, 1: distance_mm * base[1] / radius}
        else:
            deltas = {TCP_AXES.index(axis): distance_mm}
        deltas = {k: round(d, 2) for k, d in deltas.items() if abs(round(d, 2)) >= 0.01}
        if not deltas:
            raise CommandRejected("move is smaller than 0.01 mm; not moving")

        for k, d in deltas.items():
            a = TCP_AXES[k]
            op = "+" if d >= 0 else "-"
            self.expect_ok(f"EXECPTMP.{a}=PTMP.{a}{op}{abs(d):.2f}")
        target = parse_val_tuple(self.expect_ok("VALPTMP"))

        if len(target) != len(base):
            raise CommandRejected(f"unexpected VALPTMP shape: base {base}, target {target}; not moving")
        for k in range(len(base)):
            expected = base[k] + deltas.get(k, 0.0)
            tolerance = 0.05 if k in deltas else 0.005
            if abs(target[k] - expected) > tolerance:
                raise CommandRejected(
                    f"TCP target check failed: base {base}, target {target}, "
                    f"expected component {k} = {expected:.2f}; not moving"
                )
        self.expect_ok("EXECMOV PTMP")
        return {a: target[k] for k, a in enumerate(TCP_AXES)}


def is_ok_reply(raw):
    return raw.startswith("Qo")


def list_available_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# -- reply parsers ------------------------------------------------------

def _parse_semicolon_fields(raw, valid_keys):
    body = raw
    if body.startswith("Qo"):
        body = body[2:]
    body = body.lstrip("Kk")
    parts = body.strip("\r").split(";")
    result = {}
    i = 0
    while i < len(parts) - 1:
        key = parts[i]
        if key in valid_keys:
            val = parts[i + 1]
            try:
                result[key] = float(val)
            except ValueError:
                result[key] = None  # e.g. "****" for an axis not fitted
            i += 2
        else:
            i += 1
    return result


def parse_pposf(raw):
    """Parse a PPOSF reply into a dict of X/Y/Z/A/B/C floats (as present)."""
    return _parse_semicolon_fields(raw, ("X", "Y", "Z", "A", "B", "C"))


def parse_jposf(raw):
    """Parse a JPOSF reply into a dict of J1..J6 (float, or None if "****")."""
    return _parse_semicolon_fields(raw, ("J1", "J2", "J3", "J4", "J5", "J6"))


def parse_val_tuple(raw):
    """Parse the first (...) of a VAL reply into floats, e.g.
    'QoKJTMP=(-156.60,-6.49,+145.55,+0.00,-34.60,+61.33)'. Anything after the
    first closing parenthesis (such as position flags) is ignored."""
    start, end = raw.find("("), raw.find(")")
    if start < 0 or end < start:
        raise CommandRejected(f"unexpected VAL reply: {raw.strip()}")
    try:
        return [float(v) for v in raw[start + 1:end].split(",")]
    except ValueError as e:
        raise CommandRejected(f"unexpected VAL reply: {raw.strip()}") from e


def compute_target_angle(current_angle, offset_degrees):
    """Plain arithmetic helper for building a step-move target: no protocol
    knowledge here, just current + offset in degrees."""
    return current_angle + offset_degrees
