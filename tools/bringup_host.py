#!/usr/bin/env python3
"""PC-side companion for the Charger_27 bring-up firmware (see bringup/).

Needs pyserial (pip install pyserial). The firmware console is USART1 at 1 Mbaud 8N1: a 3.3 V
USB-UART adapter on SWD1 pin 3 (board TX), pin 5 (board RX) and pin 9 (GND), or the ST-Link bridge
UART once its bridge firmware is loaded.

    python tools/bringup_host.py --port COM7 info
    python tools/bringup_host.py --port COM7 cmd "i2c scan 400"
    python tools/bringup_host.py --port COM7 echo --bytes 1024
    python tools/bringup_host.py --port COM7 run --groups info,uart,i2c
    python tools/bringup_host.py --port COM7 run --groups can   # with a Kvaser on J5
    python tools/bringup_host.py --port COM7 run            # everything, with operator prompts
    python tools/bringup_host.py --port COM11 --input swd console   # type commands over SWD
    python tools/bringup_host.py --port COM11 --input swd bridge    # then PuTTY: Raw, 127.0.0.1:5025

--input swd sends what you type through the debugger instead of the UART RX wire: the bytes are
written over SWD into the firmware's bp_swd_input mailbox (STM32_Programmer_CLI, hot-plug, no reset)
and the replies are still read from the serial port. Use it when board TX works but RX does not.

The CAN group drives a Kvaser through python-can (pip install python-can; the Kvaser driver's
canlib32.dll must be installed). It checks the board's frames by ID, payload and period, counts error
frames, then sends BMS frames to the board and checks the board's count and decoded values. Default
channel 0 at 1 Mbit/s; change with --can-channel / --can-bitrate / --can-interface.

`run` sends commands, parses the firmware's "RESULT <id> <PASS|FAIL|INFO> <detail>" lines, asks the
operator to set up stimulus for the steps that need it, and writes a full serial log plus a summary.
Logs go to $CHARGER27_TEST_LOG_DIR, else the Charger_27_Test_Plans folder if it exists, else the
system temp directory - never into the repository. Exit status is 1 if any check failed.
"""

import argparse
import datetime
import os
import random
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

PROMPT = b"bringup> "
RESULT_RE = re.compile(r"^RESULT (\S+) (PASS|FAIL|INFO)\s*(.*)$")
DEFAULT_BAUD = 1_000_000
ALL_GROUPS = ["info", "uart", "i2c", "can", "sdc", "pp", "gpio", "cp"]
GEN_PINS = {"GEN_MCU1": "J2-19", "GEN_MCU2": "J2-17", "GEN_MCU3": "J2-6", "GEN_MCU4": "J2-8"}
SDC_PINS = {1: "J2-12", 2: "J2-14", 3: "J2-10", 4: "J2-16"}

# Charger CAN groups (go4-25 network). Payloads: chargingPowerLimit is a 16-bit big-endian watt
# value, trailing zero bytes are trimmed from the DLC by GopherCAN.
CAN_ID_POWER = 0x755  # chargingPowerLimit, forceEnableBalancing_state; every 100 ms
CAN_ID_SDC = 0x754    # sdcStatus17/18/19; every 250 ms
CAN_ID_BMS = 0x300    # bmsBatteryCurrent_A (scale 0.01, offset -300), bmsBatteryVoltage_V (scale 0.1)
CAN_EXPECT = {
    "pattern": {CAN_ID_POWER: (bytes.fromhex("123456"), 0.100), CAN_ID_SDC: (bytes.fromhex("A55A3C"), 0.250)},
    "tx": {CAN_ID_POWER: (bytes.fromhex("1770"), 0.100), CAN_ID_SDC: (None, 0.250)},
}
CAN_BMS_PAYLOAD = bytes.fromhex("7A020290")  # 12.34 A, 65.6 V
CAN_BMS_CURRENT = 12.34
CAN_BMS_VOLTAGE = 65.6
CAN_CAPTURE_S = 10
CAN_SEND_COUNT = 50


def default_log_dir():
    env = os.environ.get("CHARGER27_TEST_LOG_DIR")
    if env:
        return Path(env)
    plans = Path.home() / "OneDrive" / "Documents" / "Charger_27_Test_Plans"
    if plans.is_dir():
        return plans / "bringup_logs"
    return Path(tempfile.gettempdir()) / "charger27_bringup_logs"


class SerialLog:
    def __init__(self, path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.fh = open(path, "w", encoding="utf-8", newline="\n")

    def note(self, text):
        self.fh.write(f"\n### {text}\n")
        self.fh.flush()

    def rx(self, data):
        self.fh.write(data.decode("utf-8", "replace").replace("\r", ""))
        self.fh.flush()

    def tx(self, data):
        text = data.decode("utf-8", "replace") if all(b < 0x80 for b in data) and len(data) < 80 else f"<{len(data)} bytes>"
        self.fh.write(f"\n>>> {text.strip()}\n")
        self.fh.flush()

    def close(self):
        self.fh.close()


class SwdInput:
    """Console input through the debugger: writes into the firmware's bp_swd_input mailbox.

    Layout (bringup_io.c): u32 magic, u32 head (host writes), u32 tail (firmware writes), u8 buf[256].
    Head and tail are re-read on every write because a board reset puts both back to zero."""

    MAGIC = 0x49445753
    SIZE = 256
    CHUNK = 200

    def __init__(self, elf, cli="STM32_Programmer_CLI", nm="arm-none-eabi-nm"):
        self.cli = cli
        out = subprocess.run([nm, str(elf)], capture_output=True, text=True, check=True).stdout
        m = re.search(r"^([0-9a-fA-F]{8}) [DdBb] bp_swd_input$", out, re.M)
        if not m:
            raise RuntimeError(f"bp_swd_input not found in {elf}: flash a bring-up build that has the SWD mailbox")
        self.base = int(m.group(1), 16)
        magic, _, _ = self._read_header()
        if magic != self.MAGIC:
            raise RuntimeError(
                f"no mailbox at 0x{self.base:08X} on the target (read 0x{magic:08X}): the board is not running {elf.name}"
            )

    def _run(self, *args):
        r = subprocess.run([self.cli, "-c", "port=swd", "mode=HOTPLUG", *args], capture_output=True, text=True)
        if "Error" in r.stdout:
            raise RuntimeError("SWD access failed: " + " ".join(ln for ln in r.stdout.splitlines() if "Error" in ln))
        return r.stdout

    def _read_header(self):
        out = self._run("-r32", hex(self.base), "12")
        words = re.findall(r"0x[0-9A-Fa-f]{8}\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)", out)
        values = [int(v, 16) for v in " ".join(words).split()]
        if len(values) < 3:
            raise RuntimeError("could not read the SWD mailbox header")
        return values[0], values[1], values[2]

    def write(self, data):
        for start in range(0, len(data), self.CHUNK):
            chunk = data[start:start + self.CHUNK]
            for _attempt in range(50):
                _, head, tail = self._read_header()
                if self.SIZE - (head - tail) >= len(chunk):
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError("SWD mailbox full: the firmware is not consuming input")
            args = []
            for i, b in enumerate(chunk):
                args += ["-w8", hex(self.base + 12 + (head + i) % self.SIZE), hex(b)]
            args += ["-w32", hex(self.base + 4), hex((head + len(chunk)) & 0xFFFFFFFF)]
            self._run(*args)


class Board:
    """Line-oriented access to the firmware console. `ser` is any pyserial-like object; `tx`, when
    given, replaces ser.write for everything sent to the board (used for SWD input)."""

    def __init__(self, ser, log, tx=None):
        self.ser = ser
        self.log = log
        self.tx = tx or ser.write
        self.input_is_swd = tx is not None
        self.pending = bytearray()

    def _fill(self, deadline, quiet=False):
        if time.monotonic() >= deadline:
            raise TimeoutError
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if chunk:
            if not quiet:
                self.log.rx(chunk)
            self.pending += chunk

    def read_until(self, token, timeout, quiet=False):
        deadline = time.monotonic() + timeout
        while token not in self.pending:
            self._fill(deadline, quiet)
        end = self.pending.index(token) + len(token)
        data = bytes(self.pending[:end])
        del self.pending[:end]
        return data

    def read_exact(self, count, timeout):
        """Return up to `count` bytes (binary data, not logged); fewer if the timeout expires first."""
        deadline = time.monotonic() + timeout
        while len(self.pending) < count:
            try:
                self._fill(deadline, quiet=True)
            except TimeoutError:
                break
        data = bytes(self.pending[:count])
        del self.pending[:count]
        return data

    def write(self, data):
        self.log.tx(data)
        self.tx(data)

    def sync(self):
        self.ser.reset_input_buffer()
        self.pending.clear()
        for attempt in range(3):
            self.write(b"\n")
            try:
                self.read_until(PROMPT, 1.5)
                return
            except TimeoutError:
                continue
        raise RuntimeError(
            "no 'bringup> ' prompt: check the port, 1 Mbaud 8N1, TX/RX wiring and that the bring-up image is flashed"
        )

    @staticmethod
    def _lines(raw, cmd):
        text = raw.decode("utf-8", "replace").replace("\r", "")
        lines = [ln.rstrip() for ln in text.split("\n")]
        if lines and lines[0].strip() == cmd.strip():
            lines = lines[1:]  # the firmware echoes what was typed
        lines = [ln for ln in lines if ln.strip()]
        # A stream test produces one enormous line of repeated characters; keep only its head.
        return [ln if len(ln) <= 200 else f"{ln[:40]}... ({len(ln)} chars)" for ln in lines]

    def command(self, cmd, timeout=15.0, quiet=False):
        """Run one command to completion; return (output lines, results).

        quiet: the command emits a high-volume stream, so log a short summary instead of every byte."""
        self.write(cmd.encode() + b"\n")
        raw = self.read_until(PROMPT, timeout, quiet)[: -len(PROMPT)]
        lines = self._lines(raw, cmd)
        if quiet:
            self.log.note(f"{cmd}: " + " | ".join(lines))
        return lines, parse_results(lines)

    def command_for(self, cmd, seconds):
        """Run a command that only stops on a keypress for `seconds`, then stop it."""
        self.write(cmd.encode() + b"\n")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                self._fill(end)
            except TimeoutError:
                break
        self.write(b"q")
        raw = self.read_until(PROMPT, 15)[: -len(PROMPT)]
        lines = self._lines(raw, cmd)
        return lines, parse_results(lines)


def parse_results(lines):
    out = []
    for line in lines:
        m = RESULT_RE.match(line.strip())
        if m:
            out.append((m.group(1), m.group(2), m.group(3)))
    return out


class Session:
    def __init__(self, board, interactive):
        self.board = board
        self.interactive = interactive
        self.results = []

    def record(self, rid, verdict, detail, source="firmware"):
        self.results.append((rid, verdict, detail, source))
        print(f"  {verdict:<4} {rid}  {detail}")

    def run(self, cmd, timeout=15.0, show=True, duration=None, quiet=False):
        """Run a firmware command. With `duration`, let a run-until-keypress command go that long."""
        print(f"> {cmd}")
        if duration is None:
            lines, results = self.board.command(cmd, timeout, quiet)
        else:
            lines, results = self.board.command_for(cmd, duration)
        if show:
            for line in lines:
                if not line.startswith("RESULT "):
                    print(f"    {line}")
        for rid, verdict, detail in results:
            self.record(rid, verdict, detail)
        return results

    def ask(self, message):
        """Operator step. Returns False (and records a skip) when running non-interactively."""
        if not self.interactive:
            self.record("operator", "INFO", f"skipped (non-interactive): {message}", "host")
            return False
        input(f"\n  ACTION: {message}\n  press Enter when ready ")
        return True

    def confirm_yes(self, question):
        """Plain yes/no from the operator with nothing recorded; False when non-interactive."""
        if not self.interactive:
            return False
        while True:
            ans = input(f"\n  {question} [y/n] ").strip().lower()
            if ans in ("y", "n"):
                return ans == "y"

    def confirm(self, rid, question):
        if not self.interactive:
            self.record(rid, "INFO", f"skipped (non-interactive): {question}", "operator")
            return
        yes = self.confirm_yes(f"CHECK: {question}")
        self.record(rid, "PASS" if yes else "FAIL", question, "operator")

    def measure(self, rid, prompt):
        if not self.interactive:
            return
        value = input(f"  {prompt} (blank to skip): ").strip()
        if value:
            self.record(rid, "INFO", value, "operator")


def echo_block(board, session, label, payload):
    board.write(b"uart echo\n")
    board.read_until(b"ECHO READY", 5)
    board.read_until(b"\n", 1)
    board.write(payload)
    echoed = board.read_exact(len(payload), 2 + len(payload) / 20000)
    board.log.note(f"echo data: sent {len(payload)} bytes, received {len(echoed)} bytes (binary, not logged)")
    bad = [i for i, (a, b) in enumerate(zip(payload, echoed)) if a != b]
    ok = echoed == payload
    detail = f"{label}: sent {len(payload)}, echoed {len(echoed)}, wrong bytes {len(bad)}"
    if bad:
        i = bad[0]
        detail += f", first at {i} (sent 0x{payload[i]:02X} got 0x{echoed[i]:02X})"
    session.record("uart.echo.data", "PASS" if ok else "FAIL", detail, "host")
    raw = board.read_until(PROMPT, 40).decode("utf-8", "replace").replace("\r", "")
    for rid, verdict, text in parse_results(raw.split("\n")):
        session.record(rid, verdict, text)


def group_info(s):
    s.run("info")


def group_uart(s, nbytes):
    print("\n-- UART echo (also the 'U' receive check) --")
    if s.board.input_is_swd:
        s.record("uart.echo", "INFO", "skipped: input is going over SWD, so the UART RX path is not being tested", "host")
    else:
        echo_block(s.board, s, "'U' block", b"U" * nbytes)
        echo_block(s.board, s, "random block", random.Random(0x5EED).randbytes(nbytes))
    if s.ask("Optional: scope U1 pin 43 / SWD1 pin 3 now. The next step streams 0x55 for 5 s"):
        s.run("uart u 5", timeout=15, quiet=True)
        s.confirm("uart.u.scope", "Even square wave at the baud rate (500 kHz), eye open, no overshoot?")


def group_i2c(s):
    print("\n-- I2C3 / EEPROM --")
    s.run("i2c scan 100")
    s.run("i2c scan 400")
    s.run("i2c id")
    s.run("i2c eeprom", timeout=30)


def open_can_bus(args):
    """Return (bus, None) or (None, reason). Separate so tests can substitute a simulated bus."""
    try:
        import can
    except ImportError:
        return None, "python-can is not installed (pip install python-can)"
    channel = args.can_channel
    if channel is None:
        # The Kvaser driver always exposes two "Virtual CAN Driver" channels; they are a software
        # loopback, so testing against one would pass or fail without the board being involved.
        configs = can.detect_available_configs(interfaces=[args.can_interface])
        real = [c for c in configs if "virtual" not in str(c.get("device_name", "")).lower()]
        if not real:
            names = ", ".join(sorted({str(c.get("device_name")) for c in configs})) or "none"
            return None, f"no physical {args.can_interface} interface found (channels seen: {names})"
        channel = real[0]["channel"]
        print(f"CAN: using {real[0].get('device_name')} (channel {channel})")
    try:
        bus = can.Bus(interface=args.can_interface, channel=channel, bitrate=args.can_bitrate)
    except Exception as exc:  # noqa: BLE001 - any driver failure means "no bus", report it
        return None, f"could not open {args.can_interface} channel {args.can_channel}: {exc}"
    return bus, None


class CanCapture:
    """Drains a python-can bus on a thread so frames are not lost while the serial side is busy."""

    def __init__(self, bus):
        self.bus = bus
        self.frames = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            msg = self.bus.recv(0.05)
            if msg is not None:
                self.frames.append(msg)

    def __enter__(self):
        while self.bus.recv(0) is not None:  # discard anything queued before the test
            pass
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(2)


def check_frames(session, label, frames, expect, seconds):
    errors = [m for m in frames if m.is_error_frame]
    data_frames = [m for m in frames if not m.is_error_frame]
    seen_ids = {m.arbitration_id for m in data_frames}
    unexpected = sorted(seen_ids - set(expect))

    session.record(f"can.{label}.errors", "PASS" if not errors else "FAIL",
                   f"{len(errors)} error frames in {len(frames)} frames over {seconds} s", "host")
    for can_id, (payload, period) in expect.items():
        mine = [m for m in data_frames if m.arbitration_id == can_id]
        rid = f"can.{label}.0x{can_id:03X}"
        if not mine:
            session.record(rid, "FAIL", "no frames received", "host")
            continue
        expected_n = seconds / period
        count_ok = 0.8 * expected_n <= len(mine) <= 1.2 * expected_n
        gaps = [b.timestamp - a.timestamp for a, b in zip(mine, mine[1:])]
        mean_ms = statistics.mean(gaps) * 1000 if gaps else 0.0
        max_ms = max(gaps) * 1000 if gaps else 0.0
        payloads = {bytes(m.data) for m in mine}
        if payload is None:
            payload_ok = all(len(m.data) <= 3 for m in mine)
            payload_text = "payloads " + ", ".join(p.hex(" ").upper() or "(empty)" for p in sorted(payloads))
        else:
            payload_ok = payloads == {payload}
            payload_text = f"payload {'ok' if payload_ok else 'WRONG: ' + ', '.join(p.hex(' ').upper() for p in sorted(payloads))}"
        ok = count_ok and payload_ok and max_ms < 3 * period * 1000
        session.record(rid, "PASS" if ok else "FAIL",
                       f"{len(mine)} frames (expected about {expected_n:.0f}), mean period {mean_ms:.1f} ms, "
                       f"max gap {max_ms:.1f} ms, {payload_text}", "host")
    if unexpected:
        session.record(f"can.{label}.other_ids", "INFO",
                       "also saw " + ", ".join(f"0x{i:03X}" for i in unexpected), "host")


def can_board_to_host(session, bus, mode):
    cmd = f"can {mode} {CAN_CAPTURE_S}"
    print(f"> {cmd}  (capturing on the Kvaser)")
    with CanCapture(bus) as cap:
        lines, results = session.board.command(cmd, timeout=CAN_CAPTURE_S + 15)
    for line in lines:
        if not line.startswith("RESULT ") and line.startswith("  0x"):
            print(f"    {line}")
    for rid, verdict, detail in results:
        session.record(rid, verdict, detail)
    check_frames(session, mode, cap.frames, CAN_EXPECT[mode], CAN_CAPTURE_S)


def can_host_to_board(session, bus):
    import can

    seconds = 5
    board = session.board
    print(f"> can rx {seconds}  (sending {CAN_SEND_COUNT} x 0x{CAN_ID_BMS:03X} from the Kvaser)")
    board.write(f"can rx {seconds}\n".encode())
    time.sleep(0.5)
    sent = 0
    for _ in range(CAN_SEND_COUNT):
        try:
            bus.send(can.Message(arbitration_id=CAN_ID_BMS, data=CAN_BMS_PAYLOAD, is_extended_id=False))
            sent += 1
        except can.CanError as exc:
            session.record("can.rx.send", "FAIL", f"Kvaser send failed after {sent}: {exc}", "host")
            break
        time.sleep(0.02)
    raw = board.read_until(PROMPT, seconds + 15)[: -len(PROMPT)]
    results = parse_results(board._lines(raw, f"can rx {seconds}"))
    for rid, verdict, detail in results:
        session.record(rid, verdict, detail)
        if rid != "can.rx":
            continue
        fields = dict(item.split("=") for item in detail.split() if "=" in item)
        got = int(fields.get("rx_frames", "0"))
        current = float(fields.get("current", "nan"))
        voltage = float(fields.get("voltage", "nan"))
        count_ok = got == sent
        decode_ok = abs(current - CAN_BMS_CURRENT) < 0.02 and abs(voltage - CAN_BMS_VOLTAGE) < 0.2
        session.record("can.rx.count", "PASS" if count_ok else "FAIL", f"sent {sent}, board counted {got}", "host")
        session.record("can.rx.decode", "PASS" if decode_ok else "FAIL",
                       f"board decoded {current:.2f} A / {voltage:.2f} V, expected {CAN_BMS_CURRENT} / {CAN_BMS_VOLTAGE}",
                       "host")


def group_can(s, args):
    print("\n-- CAN2 --")
    s.run("can stat")
    bus, why = open_can_bus(args)
    if bus is None:
        s.record("can.kvaser", "INFO", f"no CAN interface on the PC ({why}); falling back to operator checks", "host")
        if not s.ask("Connect ONLY the CAN analyzer (1 Mbit/s, bus terminated to ~60 ohm) to J5 and start capturing"):
            return
        s.run("can tx 10", timeout=30)
        s.confirm("can.frames.live", "Analyzer saw 0x755 [17 70] every ~100 ms and 0x754 every ~250 ms, zero error frames?")
        return

    try:
        s.ask("Kvaser on J5 (pin 7 CANH, pin 2 CANL, pin 6 GND) with a 120 ohm terminator at the Kvaser end; "
              "nothing else on the bus")
        can_board_to_host(s, bus, "tx")
        if s.interactive and s.confirm_yes("Run the fixed-pattern frames? Only if NO BMS or car is on this bus"):
            can_board_to_host(s, bus, "pattern")
        else:
            s.record("can.pattern", "INFO", "skipped (needs explicit confirmation that only the Kvaser is on the bus)", "host")
        can_host_to_board(s, bus)
    finally:
        bus.shutdown()


def group_sdc(s):
    print("\n-- Shutdown circuit sense --")
    if s.ask("Hold all four SDC inputs (J2-12, 14, 10, 16) at 0 V / open"):
        s.run("sdc expect 0000")
    if s.ask("Energize all four SDC inputs at the SDC voltage"):
        s.run("sdc expect 1111")
    for ch, pin in SDC_PINS.items():
        bits = "".join("1" if i == ch else "0" for i in range(1, 5))
        if s.ask(f"Energize only SDC{ch} ({pin}); hold the other three at 0 V"):
            s.run(f"sdc expect {bits}")


def group_pp(s):
    print("\n-- Proximity pilot --")
    for token, text in (("open", "PP open (nothing connected)"), ("150", "150 ohm from PP to GND"), ("480", "480 ohm from PP to GND")):
        if s.ask(text):
            s.run(f"pp expect {token}")


def group_gpio(s):
    print("\n-- GPIO --")
    if s.ask("Watch the LEDs; the next step lights each one in turn"):
        s.run("gpio walk 800", timeout=30)
        s.confirm("gpio.leds", "Each LED lit in the printed order (LD1 green, LD2 blue, LD3 red, D3 segments 1-3)?")
    for name, pin in GEN_PINS.items():
        if s.ask(f"Drive {name} ({pin}) LOW from an external source"):
            s.run(f"gpio in {name}", show=False)
            s.run(f"gpio expect {name} 0")
        if s.ask(f"Drive {name} ({pin}) HIGH (3.3 V) from an external source"):
            s.run(f"gpio expect {name} 1")
    for name, pin in GEN_PINS.items():
        if s.ask(f"DISCONNECT the external source from {name} ({pin}) (the next step drives it), then put a DMM on it"):
            s.run(f"gpio out {name} 1", show=False)
            s.measure(f"gpio.{name}.high_V", f"{name} high, volts")
            s.run(f"gpio out {name} 0", show=False)
            s.measure(f"gpio.{name}.low_V", f"{name} low, volts")
            s.run(f"gpio in {name}", show=False)


def group_cp(s):
    print("\n-- J1772 control pilot (needs a 1 kHz +/-12 V EVSE simulator on CP) --")
    for duty in (10, 25, 50, 85):
        if not s.ask(f"Set the simulator to 1 kHz at {duty} % duty"):
            continue
        results = s.run("cp")
        for rid, _verdict, detail in results:
            if rid != "cp.read":
                continue
            fields = dict(item.split("=") for item in detail.split() if "=" in item)
            if fields.get("present") != "1":
                s.record("cp.duty", "FAIL", f"{duty} %: no pilot captured", "host")
                continue
            err = float(fields["duty"]) - duty
            hz = float(fields["hz"])
            ok = abs(err) <= 1.0 and abs(hz - 1000.0) <= 20.0
            s.record("cp.duty", "PASS" if ok else "FAIL", f"{duty} % applied: read {fields['duty']} % at {hz} Hz (error {err:+.1f} points)", "host")
    if s.ask("Start the pilot at 25 % duty. After you press Enter you have 30 s to remove it"):
        s.run("cp auto", duration=30)
        s.confirm("cp.loss", "A 'pilot lost' line appeared with CP_EN dropping about 10 ms after the last edge?")


def run_groups(s, groups, args):
    nbytes = args.bytes
    handlers = {
        "info": lambda: group_info(s),
        "uart": lambda: group_uart(s, nbytes),
        "i2c": lambda: group_i2c(s),
        "can": lambda: group_can(s, args),
        "sdc": lambda: group_sdc(s),
        "pp": lambda: group_pp(s),
        "gpio": lambda: group_gpio(s),
        "cp": lambda: group_cp(s),
    }
    for name in groups:
        s.board.log.note(f"group {name}")
        handlers[name]()


def summarize(session, log_path):
    print("\n==== summary ====")
    width = max((len(r[0]) for r in session.results), default=10)
    for rid, verdict, detail, source in session.results:
        print(f"{verdict:<4} {rid:<{width}}  [{source}] {detail}")
    fails = [r for r in session.results if r[1] == "FAIL"]
    passes = [r for r in session.results if r[1] == "PASS"]
    print(f"\n{len(passes)} passed, {len(fails)} failed, {len(session.results) - len(passes) - len(fails)} info")
    print(f"log: {log_path}")
    return 1 if fails else 0


def interactive_console(board):
    """Type commands, see replies. Output is printed as it arrives; input goes through board.tx."""
    ser = board.ser
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                board.log.rx(chunk)
                sys.stdout.write(chunk.decode("utf-8", "replace").replace("\r", ""))
                sys.stdout.flush()

    print("console: type a command and press Enter; type anything + Enter to stop a running test; Ctrl+C quits")
    if board.input_is_swd:
        print("(input goes over SWD, so each line takes a fraction of a second to arrive)")
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    board.write(b"\n")
    try:
        while True:
            line = input()
            board.write(line.encode() + b"\n")
    except (KeyboardInterrupt, EOFError):
        print("\nconsole closed")
    finally:
        stop.set()
        t.join(1)


def bridge(board, port):
    """Let a terminal (PuTTY, Raw connection) drive the board through 127.0.0.1:<port>.

    Board output from the serial port goes to the terminal; keystrokes from the terminal go to the
    board through board.tx (the SWD mailbox with --input swd). Keystrokes are batched so a burst of
    typing costs one SWD write rather than one per key."""
    import socket

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    print(f"bridge: waiting for a terminal on 127.0.0.1:{port} (PuTTY: Raw, that host and port). Ctrl+C quits")
    while True:
        conn, _ = srv.accept()
        conn.settimeout(0.05)
        print("bridge: terminal connected")
        stop = threading.Event()

        def board_to_terminal():
            while not stop.is_set():
                chunk = board.ser.read(board.ser.in_waiting or 1)
                if chunk:
                    board.log.rx(chunk)
                    try:
                        conn.sendall(chunk)
                    except OSError:
                        stop.set()

        t = threading.Thread(target=board_to_terminal, daemon=True)
        t.start()
        try:
            while not stop.is_set():
                try:
                    data = conn.recv(256)
                except socket.timeout:
                    continue
                if not data:
                    break
                time.sleep(0.03)  # gather the rest of a fast-typed burst
                try:
                    conn.setblocking(False)
                    while True:
                        more = conn.recv(256)
                        if not more:
                            break
                        data += more
                except (BlockingIOError, OSError):
                    pass
                finally:
                    conn.settimeout(0.05)
                data = bytes(b for b in data if b != 0)  # drop telnet-style NULs some terminals send
                if data:
                    board.write(data)
        except OSError:
            pass
        finally:
            stop.set()
            t.join(1)
            conn.close()
            print("bridge: terminal disconnected, waiting again")


def open_session(args):
    import serial  # imported late so --help works without pyserial

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else default_log_dir()
    log_path = log_dir / f"bringup_{stamp}.log"
    tx = None
    if args.input == "swd":
        if not shutil.which("STM32_Programmer_CLI"):
            raise SystemExit("--input swd needs STM32_Programmer_CLI on PATH (STM32CubeCLT)")
        tx = SwdInput(Path(args.elf)).write
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    board = Board(ser, SerialLog(log_path), tx)
    if args.action not in ("console", "bridge"):
        board.sync()
    return Session(board, not args.non_interactive), log_path


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port, e.g. COM7")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--log-dir", help="where to write logs (default: see module docstring)")
    p.add_argument("--non-interactive", action="store_true", help="skip every step that needs the operator")
    p.add_argument("--can-interface", default="kvaser", help="python-can interface name (default kvaser)")
    p.add_argument("--can-channel", type=int, default=None,
                   help="CAN channel number (default: the first non-virtual channel found)")
    p.add_argument("--can-bitrate", type=int, default=1_000_000)
    p.add_argument("--input", choices=("uart", "swd"), default="uart",
                   help="how commands reach the board: the UART RX wire, or the debugger (SWD) mailbox")
    p.add_argument("--elf", default=str(Path(__file__).resolve().parent.parent / "build" / "Bringup" / "Charger_27_bringup.elf"),
                   help="bring-up ELF on the board, used to locate the SWD mailbox")
    sub = p.add_subparsers(dest="action", required=True)
    sub.add_parser("console", help="interactive terminal: type commands, see replies")
    b = sub.add_parser("bridge", help="serve the console to PuTTY (Raw) on 127.0.0.1")
    b.add_argument("--tcp-port", type=int, default=5025)
    sub.add_parser("info", help="print MCU ID, clocks and reset cause")
    c = sub.add_parser("cmd", help="run one firmware command and show its output")
    c.add_argument("text")
    e = sub.add_parser("echo", help="send a block and compare the echo")
    e.add_argument("--bytes", type=int, default=1024)
    r = sub.add_parser("run", help="run test groups")
    r.add_argument("--groups", default=",".join(ALL_GROUPS), help=f"comma list of {', '.join(ALL_GROUPS)}")
    r.add_argument("--bytes", type=int, default=1024, help="echo block size")
    args = p.parse_args(argv)

    session, log_path = open_session(args)
    if args.action == "bridge":
        try:
            bridge(session.board, args.tcp_port)
        except KeyboardInterrupt:
            print("\nbridge stopped")
        finally:
            session.board.log.close()
        return 0
    if args.action == "console":
        try:
            interactive_console(session.board)
        finally:
            session.board.log.close()
            print(f"log: {log_path}")
        return 0
    try:
        if args.action == "info":
            group_info(session)
        elif args.action == "cmd":
            session.run(args.text, timeout=60)
        elif args.action == "echo":
            group_uart(session, args.bytes)
        else:
            groups = [g.strip() for g in args.groups.split(",") if g.strip()]
            unknown = [g for g in groups if g not in ALL_GROUPS]
            if unknown:
                p.error(f"unknown group(s): {', '.join(unknown)}")
            run_groups(session, groups, args)
    finally:
        status = summarize(session, log_path)
        session.board.log.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
