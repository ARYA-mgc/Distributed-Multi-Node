#!/usr/bin/env python3
"""
sensor_net_monitor.py
Distributed Sensor Network — Serial Monitor & Live Dashboard

Author: ARYA mgc
Version: 1.0.0
Date: 2025

Usage:
    python sensor_net_monitor.py --port COM3 --baud 115200
    python sensor_net_monitor.py --port /dev/ttyUSB0

Connects to the STM32 central node debug UART (USART3) and renders
a live table of all node states, sensor values, and fault history.
"""

import argparse
import serial
import threading
import time
import re
import sys
from datetime import datetime
from collections import defaultdict, deque

try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False

# ── Node state mapping ────────────────────────────────────────
NODE_STATES = {
    0: ("INIT",        "yellow"),
    1: ("REGISTERING", "cyan"),
    2: ("ACTIVE",      "green"),
    3: ("FAULT",       "red"),
    4: ("RECOVERING",  "magenta"),
    5: ("SAFE MODE",   "yellow"),
    6: ("OFFLINE",     "red"),
}

FAULT_NAMES = {
    0x00: "NONE",
    0x01: "SENSOR_TIMEOUT",
    0x02: "OUT_OF_RANGE",
    0x03: "CHECKSUM_ERROR",
    0x04: "COMM_ERROR",
    0x05: "POWER_ANOMALY",
    0x06: "TEMP_CRITICAL",
    0x07: "VIBRATION_HIGH",
    0x08: "GAS_LEAK",
    0x09: "NODE_OFFLINE",
    0x0A: "MEMORY_OVERFLOW",
    0x0B: "WATCHDOG_RESET",
}

RECOVERY_NAMES = {
    0x01: "RESET_SENSOR",
    0x02: "SOFT_RESET",
    0x03: "RECALIBRATE",
    0x04: "ENTER_SAFE",
    0x05: "EXIT_SAFE",
    0x06: "CLEAR_FAULT",
    0x07: "UPDATE_RATE",
    0x0F: "FACTORY_RESET",
}

# ── In-memory state ───────────────────────────────────────────

class NodeData:
    def __init__(self, node_id: str):
        self.node_id        = node_id
        self.name           = f"Node-{node_id}"
        self.state          = 0
        self.state_str      = "UNKNOWN"
        self.temperature    = None
        self.humidity       = None
        self.pressure       = None
        self.vibration      = None
        self.voltage        = None
        self.current        = None
        self.gas_ppm        = None
        self.last_seen      = None
        self.fault_code     = 0x00
        self.recovery_cnt   = 0
        self.fault_history  = deque(maxlen=10)


nodes: dict[str, NodeData] = {}
fault_log: deque = deque(maxlen=50)
raw_lines: deque = deque(maxlen=20)
lock = threading.Lock()

# ── Log line parsers ──────────────────────────────────────────

DATA_RE      = re.compile(
    r"\[DATA\] Node (0x\w+) \| T=([\-\d.]+).*?H=([\d.]+).*?P=(\d+).*?"
    r"V=([\-\d]+)mg.*?Vcc=(\d+).*?I=(\d+).*?Gas=(\d+)")
HB_RE        = re.compile(r"\[HB\] Node (0x\w+) \| state=(\d) fault_cnt=(\d+)")
REG_RE       = re.compile(r"Node (0x\w+) '(\w[\w\-]*)' registered")
FAULT_RE     = re.compile(r"\[FAULT\] Node (0x\w+):? (.+)")
RECOVERY_RE  = re.compile(r"\[RECOVERY\].* Node (0x\w+) (\w.+)")
OFFLINE_RE   = re.compile(r"Node (0x\w+) OFFLINE")


def parse_line(line: str):
    with lock:
        raw_lines.append(line.strip())

        m = DATA_RE.search(line)
        if m:
            nid = m.group(1)
            if nid not in nodes: nodes[nid] = NodeData(nid)
            n = nodes[nid]
            n.temperature = float(m.group(2))
            n.humidity    = float(m.group(3))
            n.pressure    = int(m.group(4))
            n.vibration   = int(m.group(5))
            n.voltage     = int(m.group(6))
            n.current     = int(m.group(7))
            n.gas_ppm     = int(m.group(8))
            n.last_seen   = datetime.now().strftime("%H:%M:%S")
            return

        m = HB_RE.search(line)
        if m:
            nid, state = m.group(1), int(m.group(2))
            if nid not in nodes: nodes[nid] = NodeData(nid)
            nodes[nid].state     = state
            nodes[nid].state_str = NODE_STATES.get(state, ("?", "white"))[0]
            nodes[nid].last_seen = datetime.now().strftime("%H:%M:%S")
            return

        m = REG_RE.search(line)
        if m:
            nid, name = m.group(1), m.group(2)
            if nid not in nodes: nodes[nid] = NodeData(nid)
            nodes[nid].name      = name
            nodes[nid].state_str = "ACTIVE"
            nodes[nid].last_seen = datetime.now().strftime("%H:%M:%S")
            return

        m = FAULT_RE.search(line)
        if m:
            nid = m.group(1)
            if nid not in nodes: nodes[nid] = NodeData(nid)
            desc = m.group(2)
            nodes[nid].state_str = "FAULT"
            ts = datetime.now().strftime("%H:%M:%S")
            entry = f"[{ts}] {nid}: {desc}"
            fault_log.appendleft(entry)
            nodes[nid].fault_history.appendleft(entry)
            return

        m = OFFLINE_RE.search(line)
        if m:
            nid = m.group(1)
            if nid not in nodes: nodes[nid] = NodeData(nid)
            nodes[nid].state_str = "OFFLINE"
            return

        m = RECOVERY_RE.search(line)
        if m:
            nid = m.group(1)
            if nid not in nodes: nodes[nid] = NodeData(nid)
            nodes[nid].state_str  = "RECOVERING"
            nodes[nid].recovery_cnt += 1


# ── Serial reader thread ──────────────────────────────────────

def serial_reader(port: str, baud: int):
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                print(f"[MONITOR] Connected to {port} @ {baud}")
                while True:
                    line = ser.readline().decode("utf-8", errors="replace")
                    if line:
                        parse_line(line)
        except serial.SerialException as e:
            print(f"[MONITOR] Serial error: {e} — retrying in 3s")
            time.sleep(3)


# ── Rich dashboard ────────────────────────────────────────────

def build_dashboard() -> Layout:
    layout = Layout()
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="body"),
        Layout(name="footer", size=12),
    )
    layout["body"].split_row(
        Layout(name="nodes", ratio=2),
        Layout(name="faults", ratio=1),
    )

    # Header
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    layout["header"].update(Panel(
        Text(f"  Distributed Sensor Network Monitor — ARYA mgc   [{ts}]",
             style="bold cyan"),
        style="cyan"
    ))

    # Node table
    with lock:
        table = Table(title="Node Status", expand=True, border_style="cyan")
        table.add_column("ID",       style="bold white",  width=10)
        table.add_column("Name",     style="white",       width=14)
        table.add_column("State",    width=12)
        table.add_column("T (°C)",   justify="right",     width=8)
        table.add_column("H (%)",    justify="right",     width=7)
        table.add_column("P (Pa)",   justify="right",     width=9)
        table.add_column("Vib (mg)", justify="right",     width=9)
        table.add_column("Vcc (mV)", justify="right",     width=9)
        table.add_column("I (mA)",   justify="right",     width=8)
        table.add_column("Gas ppm",  justify="right",     width=9)
        table.add_column("Seen",     width=10)

        for nid, n in sorted(nodes.items()):
            _, clr = NODE_STATES.get(n.state, ("?", "white"))
            table.add_row(
                nid,
                n.name,
                Text(n.state_str, style=f"bold {clr}"),
                f"{n.temperature:.1f}" if n.temperature is not None else "—",
                f"{n.humidity:.0f}"    if n.humidity    is not None else "—",
                str(n.pressure)        if n.pressure    is not None else "—",
                str(n.vibration)       if n.vibration   is not None else "—",
                str(n.voltage)         if n.voltage     is not None else "—",
                str(n.current)         if n.current     is not None else "—",
                str(n.gas_ppm)         if n.gas_ppm     is not None else "—",
                n.last_seen or "—",
            )

        layout["nodes"].update(Panel(table, title="Nodes", border_style="cyan"))

        # Fault log
        fault_text = "\n".join(list(fault_log)[:15]) or "No faults recorded."
        layout["faults"].update(Panel(
            Text(fault_text, style="red"),
            title="Fault Log", border_style="red"
        ))

        # Raw log footer
        raw_text = "\n".join(list(raw_lines)[-10:])
        layout["footer"].update(Panel(
            Text(raw_text, style="dim"),
            title="Raw UART Log", border_style="dim"
        ))

    return layout


def run_dashboard(port: str, baud: int):
    t = threading.Thread(target=serial_reader, args=(port, baud), daemon=True)
    t.start()
    console = Console()
    with Live(console=console, refresh_per_second=2, screen=True) as live:
        while True:
            live.update(build_dashboard())
            time.sleep(0.5)


def run_plain(port: str, baud: int):
    t = threading.Thread(target=serial_reader, args=(port, baud), daemon=True)
    t.start()
    print(f"[MONITOR] Plain mode — press Ctrl+C to exit")
    try:
        while True:
            time.sleep(1)
            with lock:
                print(f"\n=== {datetime.now().strftime('%H:%M:%S')} ===")
                for nid, n in sorted(nodes.items()):
                    print(f"  {nid} [{n.state_str:12}] T={n.temperature}°C "
                          f"H={n.humidity}% P={n.pressure}Pa gas={n.gas_ppm}ppm")
    except KeyboardInterrupt:
        pass


# ── Entry point ───────────────────────────────────────────────

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Sensor Network Monitor — ARYA mgc")
    ap.add_argument("--port",  default="COM3",   help="Serial port")
    ap.add_argument("--baud",  type=int, default=115200, help="Baud rate")
    ap.add_argument("--plain", action="store_true", help="No TUI (plain output)")
    args = ap.parse_args()

    if RICH_AVAILABLE and not args.plain:
        run_dashboard(args.port, args.baud)
    else:
        if not RICH_AVAILABLE:
            print("[MONITOR] Install 'rich' for TUI: pip install rich")
        run_plain(args.port, args.baud)
