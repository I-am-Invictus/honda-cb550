#!/usr/bin/env python3
"""
Delta-Q IC/RC/ICL CANopen simplified open-loop charge controller.

This script uses python-can with a SocketCAN-compatible USB CAN adapter
(e.g. candleLight, gs_usb adapter exposed as can0, PCAN via socketcan, etc.)
to emulate the battery/BMS side enough to command the charger in remote mode.

WHAT IT DOES
- Sends BMS heartbeat (node 1 -> charger)
- Sends NMT start for charger node 10
- Sends RPDO1 at a fixed rate until cancelled
- First sends Battery Status = 0, then Battery Status = 1
- On Ctrl+C, requests 0 current, then Battery Status = 0, then exits
- Prints charger TPDO1 feedback (current/voltage/status) as received

IMPORTANT
This is open-loop charging. You are bypassing normal BMS control.
Use at your own risk and only with independent pack/cell monitoring.
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass

import can


@dataclass(frozen=True)
class ChargeConfig:
    channel: str = "can0"
    bustype: str = "socketcan"
    bitrate: int = 500000
    charger_node_id: int = 0x0A
    battery_node_id: int = 0x01
    pack_voltage_v: float = 82.0
    charge_current_a: float = 10.0
    send_period_s: float = 0.25
    startup_delay_s: float = 1.0
    shutdown_hold_s: float = 0.5
    print_feedback: bool = True


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def voltage_to_2276_bytes(volts: float) -> bytes:
    # Object 2276h scaling: 1/256 V, little-endian UNSIGNED16
    raw = int(round(volts * 256.0))
    if not 0 <= raw <= 0xFFFF:
        raise ValueError(f"Voltage request out of range for 2276h: {volts} V")
    return struct.pack("<H", raw)


def current_to_6070_bytes(amps: float) -> bytes:
    # Object 6070h scaling: 1/16 A, little-endian UNSIGNED16
    raw = int(round(amps * 16.0))
    if not 0 <= raw <= 0xFFFF:
        raise ValueError(f"Current request out of range for 6070h: {amps} A")
    return struct.pack("<H", raw)


def build_rpdo1(voltage_v: float, current_a: float, battery_ready: bool, soc_pct: int = 0, external_override: int = 0) -> bytes:
    if not 0 <= soc_pct <= 100:
        raise ValueError("soc_pct must be 0..100")
    if not 0 <= external_override <= 0xFF:
        raise ValueError("external_override must be 0..255")

    voltage = voltage_to_2276_bytes(voltage_v)
    current = current_to_6070_bytes(current_a)
    battery_status = 1 if battery_ready else 0

    # RPDO1 layout from the Delta-Q app note:
    # byte 0: zero fill
    # byte 1: 6081h Battery SOC
    # byte 2: 4201h External Override 0
    # bytes 3-4: 2276h Voltage Request
    # bytes 5-6: 6070h Charge Current Requested
    # byte 7: 6000h Battery Status
    return bytes([
        0x00,
        soc_pct & 0xFF,
        external_override & 0xFF,
        voltage[0], voltage[1],
        current[0], current[1],
        battery_status,
    ])


def build_can_ids(charger_node_id: int, battery_node_id: int) -> dict[str, int]:
    return {
        "nmt": 0x000,
        "heartbeat_to_charger": 0x700 + battery_node_id,
        "charger_heartbeat": 0x700 + charger_node_id,
        "rpdo1": 0x200 + charger_node_id,
        "tpdo1": 0x180 + charger_node_id,
        "tpdo2": 0x280 + charger_node_id,
        "tpdo3": 0x380 + charger_node_id,
    }


class DeltaQOpenLoopController:
    def __init__(self, bus: can.BusABC, cfg: ChargeConfig):
        self.bus = bus
        self.cfg = cfg
        self.ids = build_can_ids(cfg.charger_node_id, cfg.battery_node_id)
        self.stop_event = threading.Event()
        self.rx_thread: threading.Thread | None = None
        self.tx_thread: threading.Thread | None = None
        self.last_feedback_print = 0.0

    def send(self, arbitration_id: int, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)

    def send_nmt_start(self) -> None:
        # 01 = start remote node, second byte = charger node id
        self.send(self.ids["nmt"], bytes([0x01, self.cfg.charger_node_id]))
        print(f"TX NMT start -> node {self.cfg.charger_node_id} (ID 0x000): 01 {self.cfg.charger_node_id:02X}")

    def send_heartbeat_operational(self) -> None:
        # 05 = operational
        self.send(self.ids["heartbeat_to_charger"], bytes([0x05]))

    def send_rpdo1(self, voltage_v: float, current_a: float, battery_ready: bool) -> None:
        payload = build_rpdo1(voltage_v, current_a, battery_ready)
        self.send(self.ids["rpdo1"], payload)
        print(
            f"TX RPDO1 0x{self.ids['rpdo1']:03X}: {payload.hex(' ')} "
            f"(Vreq={voltage_v:.3f} V, Ireq={current_a:.3f} A, ready={int(battery_ready)})"
        )

    def startup_sequence(self) -> None:
        print("Sending startup sequence...")
        self.send_heartbeat_operational()
        print(f"TX heartbeat 0x{self.ids['heartbeat_to_charger']:03X}: 05")
        time.sleep(0.05)
        self.send_nmt_start()
        time.sleep(self.cfg.startup_delay_s)

        # Per Delta-Q note: send Battery Status 0 first, then 1
        self.send_rpdo1(self.cfg.pack_voltage_v, self.cfg.charge_current_a, battery_ready=False)
        time.sleep(self.cfg.send_period_s)
        self.send_rpdo1(self.cfg.pack_voltage_v, self.cfg.charge_current_a, battery_ready=True)
        print("Startup sequence complete. Entering steady-state transmit loop.")

    def shutdown_sequence(self) -> None:
        print("\nSending shutdown sequence...")
        try:
            # Recommended: set current to 0 first, then battery status to 0
            self.send_rpdo1(self.cfg.pack_voltage_v, 0.0, battery_ready=True)
            time.sleep(self.cfg.shutdown_hold_s)
            self.send_rpdo1(self.cfg.pack_voltage_v, 0.0, battery_ready=False)
            time.sleep(0.1)
            self.send_heartbeat_operational()
        except Exception as exc:
            print(f"Shutdown send error: {exc}", file=sys.stderr)
        print("Shutdown sequence sent.")

    def tx_loop(self) -> None:
        next_heartbeat = time.monotonic()
        next_rpdo = time.monotonic()
        hb_period = 1.0

        while not self.stop_event.is_set():
            now = time.monotonic()
            if now >= next_heartbeat:
                try:
                    self.send_heartbeat_operational()
                except Exception as exc:
                    print(f"Heartbeat send error: {exc}", file=sys.stderr)
                next_heartbeat += hb_period

            if now >= next_rpdo:
                try:
                    self.send(self.ids["rpdo1"], build_rpdo1(self.cfg.pack_voltage_v, self.cfg.charge_current_a, True))
                except Exception as exc:
                    print(f"RPDO1 send error: {exc}", file=sys.stderr)
                next_rpdo += self.cfg.send_period_s

            time.sleep(0.01)

    def rx_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.25)
            except Exception as exc:
                print(f"Receive error: {exc}", file=sys.stderr)
                continue

            if msg is None:
                continue

            if msg.arbitration_id == self.ids["charger_heartbeat"] and len(msg.data) >= 1:
                state = msg.data[0]
                print(f"RX heartbeat 0x{msg.arbitration_id:03X}: {state:02X}")
                continue

            if self.cfg.print_feedback and msg.arbitration_id == self.ids["tpdo1"] and len(msg.data) >= 6:
                try:
                    self.print_tpdo1(msg)
                except Exception as exc:
                    print(f"TPDO1 decode error: {exc}; raw={msg.data.hex(' ')}", file=sys.stderr)
                continue

    def print_tpdo1(self, msg: can.Message) -> None:
        # TPDO1:
        # bytes 0-1: 2002h Charging Current, 1/256 A
        # bytes 2-3: 2101h Battery Voltage, 1/256 V
        # bytes 4-5: 2006h Extended Charge Status
        current_raw = struct.unpack_from("<H", msg.data, 0)[0]
        voltage_raw = struct.unpack_from("<H", msg.data, 2)[0]
        status_raw = struct.unpack_from("<H", msg.data, 4)[0]

        current_a = current_raw / 256.0
        voltage_v = voltage_raw / 256.0

        charge_cycle_type = (status_raw >> 12) & 0x0F
        charge_indication = (status_raw >> 8) & 0x0F
        override_status = (status_raw >> 6) & 0x03
        charger_enabled = (status_raw >> 5) & 0x01
        ac_detected = (status_raw >> 4) & 0x01
        derating = (status_raw >> 3) & 0x01
        hw_shutdown = (status_raw >> 2) & 0x01
        active_alarm = (status_raw >> 1) & 0x01
        active_fault = status_raw & 0x01

        now = time.monotonic()
        if now - self.last_feedback_print >= 0.2:
            self.last_feedback_print = now
            print(
                "RX TPDO1 "
                f"0x{msg.arbitration_id:03X}: "
                f"I={current_a:.2f} A, V={voltage_v:.2f} V, "
                f"status=0x{status_raw:04X}, cycle={charge_cycle_type}, ind={charge_indication}, "
                f"override={override_status}, enabled={charger_enabled}, AC={ac_detected}, "
                f"derating={derating}, hw_shutdown={hw_shutdown}, alarm={active_alarm}, fault={active_fault}"
            )

    def start(self) -> None:
        self.rx_thread = threading.Thread(target=self.rx_loop, name="deltaq-rx", daemon=True)
        self.tx_thread = threading.Thread(target=self.tx_loop, name="deltaq-tx", daemon=True)
        self.rx_thread.start()
        self.startup_sequence()
        self.tx_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.tx_thread is not None:
            self.tx_thread.join(timeout=1.0)
        self.shutdown_sequence()
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)


def parse_args() -> ChargeConfig:
    parser = argparse.ArgumentParser(description="Open-loop Delta-Q charge control over CAN")
    parser.add_argument("--channel", default="can0", help="CAN interface/channel, default: can0")
    parser.add_argument("--bustype", default="socketcan", help="python-can bus type, default: socketcan")
    parser.add_argument("--bitrate", type=int, default=125000, help="CAN bitrate, default: 125000")
    parser.add_argument("--charger-node", type=int, default=10, help="Charger CAN node ID, default: 10")
    parser.add_argument("--battery-node", type=int, default=1, help="Battery/BMS CAN node ID, default: 1")
    parser.add_argument("--voltage", type=float, default=82.0, help="Pack voltage request in V, default: 82.0")
    parser.add_argument("--current", type=float, default=10.0, help="Charge current request in A, default: 10.0")
    parser.add_argument("--period-ms", type=float, default=250.0, help="RPDO1 repeat period in ms, default: 250")
    parser.add_argument("--startup-delay-ms", type=float, default=1000.0, help="Delay after NMT start in ms, default: 1000")
    parser.add_argument("--no-feedback", action="store_true", help="Disable TPDO1 decode/printing")
    args = parser.parse_args()

    # User requested conservative current under 15 A and 82 V max.
    if args.voltage > 82.0:
        parser.error("Refusing voltage request above 82.0 V.")
    if args.current > 15.0:
        parser.error("Refusing current request above 15.0 A.")
    if args.voltage < 0 or args.current < 0:
        parser.error("Voltage and current must be non-negative.")
    if args.period_ms <= 0 or args.period_ms > 500:
        parser.error("period-ms must be > 0 and <= 500 to stay within the charger timeout guidance.")

    return ChargeConfig(
        channel=args.channel,
        bustype=args.bustype,
        bitrate=args.bitrate,
        charger_node_id=args.charger_node,
        battery_node_id=args.battery_node,
        pack_voltage_v=args.voltage,
        charge_current_a=args.current,
        send_period_s=args.period_ms / 1000.0,
        startup_delay_s=args.startup_delay_ms / 1000.0,
        print_feedback=not args.no_feedback,
    )


def main() -> int:
    cfg = parse_args()
    print("Delta-Q open-loop CAN controller")
    print(f"  channel:       {cfg.channel}")
    print(f"  bustype:       {cfg.bustype}")
    print(f"  bitrate:       {cfg.bitrate}")
    print(f"  charger node:  {cfg.charger_node_id}")
    print(f"  battery node:  {cfg.battery_node_id}")
    print(f"  voltage req:   {cfg.pack_voltage_v:.3f} V")
    print(f"  current req:   {cfg.charge_current_a:.3f} A")
    print(f"  period:        {cfg.send_period_s * 1000.0:.1f} ms")
    print("Press Ctrl+C to stop.\n")

    try:
        bus = can.Bus(interface=cfg.bustype, channel=cfg.channel, bitrate=cfg.bitrate)
    except TypeError:
        # Backward compatibility with older python-can signatures
        bus = can.Bus(bustype=cfg.bustype, channel=cfg.channel, bitrate=cfg.bitrate)
    except Exception as exc:
        print(f"Failed to open CAN bus: {exc}", file=sys.stderr)
        return 1

    controller = DeltaQOpenLoopController(bus, cfg)

    stop_once = {"done": False}

    def request_stop(signum=None, frame=None):
        if stop_once["done"]:
            return
        stop_once["done"] = True
        print("\nStop requested...")
        controller.stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        controller.start()
        while not stop_once["done"]:
            time.sleep(0.25)
    except KeyboardInterrupt:
        request_stop()
    except Exception as exc:
        print(f"Fatal error: {exc}", file=sys.stderr)
        request_stop()
        return 1
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())