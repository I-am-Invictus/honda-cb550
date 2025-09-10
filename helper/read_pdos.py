#!/usr/bin/env python3
"""
Read and pretty-print the current RPDO configuration from a CANopen node.

- Shows COB-ID, enabled state (bit31 cleared), transmission type
- Decodes 0x160x mapping entries into human-readable names with units/scales
- Works even if some PDOs aren't present (prints a friendly note)

Usage examples:
  python3 read_rpdos.py --channel can0 --bustype socketcan --node-id 0x0A --eds deltaq.eds --num-pdos 2
"""

import argparse
import canopen

# ---------------------------------------------------------------------
# Human-readable dictionary for the OD entries you’ve mapped/seen.
# Add lines here if you start mapping more objects later.
# Format: index: (name, datatype, bits, units, to_eng_desc)
# ---------------------------------------------------------------------
OD_HUMAN = {
    0x6081: ("Battery_SOC",            "UNSIGNED8",   8,  "%",     "eng = raw"),
    0x2271: ("Voltage_Limit",          "UNSIGNED32", 32,  "V",     "eng = raw / 1024"),
    0x6070: ("Battery_Current",        "UNSIGNED16", 16,  "A",     "eng = raw / 16"),
    0x6000: ("Battery_Status",         "UNSIGNED8",   8,  "",      "device-specific"),
    0x6060: ("Batt_Battery_Voltage",   "UNSIGNED32", 32,  "V",     "eng = raw / 1024"),
    0x6010: ("Batt_Temperature",       "INTEGER16",  16,  "°C",    "eng = raw * 1"),
}

def _format_entry(idx: int, sub: int, bl: int) -> str:
    """Return a one-line human string for a single PDO mapping entry."""
    info = OD_HUMAN.get(idx)
    if info:
        name, dtype, bits_expected, units, scale_desc = info
        bits_note = "" if bits_expected == bl else f" (NOTE: mapped {bl}b, EDS says {bits_expected}b)"
        units_str = f" {units}" if units else ""
        return f"0x{idx:04X}:{sub:02X}  {name}  [{dtype}, {bl}b]{bits_note}  → {scale_desc}{units_str}"
    else:
        return f"0x{idx:04X}:{sub:02X}  (unknown)  [{bl}b]"

def read_all_rpdos(channel="can0", bustype="socketcan", node_id=0x0A, eds="deltaq.eds", num_pdos=4):
    """
    Connects to the CANopen network, reads the current RPDO mappings for the given node,
    and prints them in a human-readable format.
    """
    net = canopen.Network()
    net.connect(channel=channel, bustype=bustype)

    try:
        node = canopen.RemoteNode(node_id, eds)
        net.add_node(node)

        for rpdo_number in range(num_pdos):
            comm_idx = 0x1400 + rpdo_number
            map_idx  = 0x1600 + rpdo_number

            try:
                cobid = node.sdo[comm_idx][1].raw
                ttype = node.sdo[comm_idx][2].raw
                enabled = (cobid & 0x80000000) == 0

                count = node.sdo[map_idx][0].raw
                entries_human = []
                total_bits = 0
                for i in range(1, count + 1):
                    raw = node.sdo[map_idx][i].raw
                    idx = (raw >> 16) & 0xFFFF
                    sub = (raw >> 8) & 0xFF
                    bl  = raw & 0xFF
                    total_bits += bl
                    entries_human.append(_format_entry(idx, sub, bl))

                print(f"\nRPDO{rpdo_number+1}:")
                print(f"  Comm idx: 0x{comm_idx:04X}   Map idx: 0x{map_idx:04X}")
                print(f"  COB-ID:   0x{cobid:08X}   enabled={enabled}   TT=0x{ttype:02X}")
                print(f"  Map count: {count}   Total bits: {total_bits} (bytes: {total_bits//8})")
                if entries_human:
                    print("  Entries:")
                    for line in entries_human:
                        print(f"    • {line}")
                else:
                    print("  Entries: (none)")

            except Exception as e:
                print(f"\nRPDO{rpdo_number+1}: not available ({e})")

    finally:
        net.disconnect()

def main():
    ap = argparse.ArgumentParser(description="Read and pretty-print RPDOs from a CANopen node.")
    ap.add_argument("--channel", default="can0", help="python-can channel (default: can0)")
    ap.add_argument("--bustype", default="socketcan", help="python-can bustype (default: socketcan)")
    ap.add_argument("--node-id", type=lambda x: int(x, 0), default=0x0A, help="node id (e.g., 10 or 0x0A)")
    ap.add_argument("--eds", default="deltaq.eds", help="path to EDS file")
    ap.add_argument("--num-pdos", type=int, default=4, help="how many RPDOs to read (default: 4)")
    args = ap.parse_args()

    read_all_rpdos(
        channel=args.channel,
        bustype=args.bustype,
        node_id=args.node_id,
        eds=args.eds,
        num_pdos=args.num_pdos,
    )

if __name__ == "__main__":
    main()
