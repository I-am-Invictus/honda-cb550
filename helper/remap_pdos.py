#!/usr/bin/env python3
import argparse
import time
import threading
import struct
import can
import canopen
from canopen.sdo.exceptions import SdoAbortedError

# -----------------------------
# Object aliases (clarity only)
# -----------------------------
# 0x6070:00 is the unsigned-16 current request (A * 16 LSB) — call it Battery Current.
BATTERY_CURRENT_ENTRY = (0x6070, 0x00, 16)

# -----------------------------
# Mapping you’ll install
# -----------------------------
# RPDO1 @ 0x20A (64 bits total):
#   0x6081:00  Battery SOC (U8)                     -> 8 bits
#   0x2271:00  Voltage Limit / "Voltage Request"    -> 32 bits (V * 1024)
#   0x6070:00  Battery Current (U16)                -> 16 bits (A * 16)
#   0x6000:00  Battery Status (U8)                  -> 8 bits
RPDO1_ENTRIES = [
    (0x6081, 0x00, 8),
    (0x2271, 0x00, 32),
    BATTERY_CURRENT_ENTRY,          # was "Charge Current Request"
    (0x6000, 0x00, 8),
]

# RPDO2 @ 0x30A (64 bits total):
#   0x6060:00  Battery Voltage (U32, V * 1024)      -> 32 bits
#   0x6010:00  Battery Temperature (I16)            -> 16 bits
#   0x6070:00  Battery Current (U16)                -> 16 bits (A * 16)
RPDO2_ENTRIES = [
    (0x6060, 0x00, 32),
    (0x6010, 0x00, 16),
    BATTERY_CURRENT_ENTRY,          # was "Charge Current Request"
]

TRANSMISSION_TYPE = 0xFF  # asynchronous; change if your network uses SYNC

# -----------------------------
# Helpers
# -----------------------------
def map_entry(index: int, subindex: int, bitlen: int) -> int:
    """Encode a PDO mapping entry word."""
    return (index << 16) | (subindex << 8) | (bitlen & 0xFF)

def abort_meaning(code: int) -> str:
    if code == 0x06040041:
        return "Object cannot be mapped to the PDO (often because PDO not disabled or object not PDO-mappable)."
    return "See CiA 301 abort codes."

def remap_rpdo(node: canopen.RemoteNode, rpdo_number: int, cobid: int,
               tx_type: int, entries: list[tuple[int, int, int]]):
    """
    Remap a Receive PDO:
      rpdo_number: 0 -> RPDO1 (0x1400/0x1600), 1 -> RPDO2 (0x1401/0x1601), etc.
      cobid: desired COB-ID (bit31 will be set while editing, cleared when enabling)
      tx_type: transmission type (e.g., 0xFF)
      entries: list of (index, subindex, bit_length_bits)
    """
    comm_idx = 0x1400 + rpdo_number  # RPDO Communication Parameter
    map_idx  = 0x1600 + rpdo_number  # RPDO Mapping Parameter

    # 1) Disable the PDO via COB-ID bit31 (REQUIRED before editing mapping)
    node.sdo[comm_idx][1].raw = cobid | 0x80000000
    # 2) Set transmission type while disabled
    node.sdo[comm_idx][2].raw = tx_type

    # 3) Clear mapping, write new entries, set count
    node.sdo[map_idx][0].raw = 0
    total_bits = 0
    for i, (idx, sub, blen) in enumerate(entries, start=1):
        total_bits += blen
        if total_bits > 64:
            raise ValueError(f"RPDO{rpdo_number+1} mapping exceeds 64 bits (now at {total_bits}).")
        node.sdo[map_idx][i].raw = map_entry(idx, sub, blen)
    node.sdo[map_idx][0].raw = len(entries)

    # 4) Re-enable PDO by clearing bit31 in COB-ID
    node.sdo[comm_idx][1].raw = cobid

def verify_rpdo(node: canopen.RemoteNode, rpdo_number: int) -> tuple[int, int, list[tuple[int, int, int]]]:
    comm_idx = 0x1400 + rpdo_number
    map_idx  = 0x1600 + rpdo_number
    cobid = node.sdo[comm_idx][1].raw
    ttype = node.sdo[comm_idx][2].raw
    count = node.sdo[map_idx][0].raw
    entries = []
    for i in range(1, count + 1):
        raw = node.sdo[map_idx][i].raw
        idx = (raw >> 16) & 0xFFFF
        sub = (raw >> 8) & 0xFF
        bl  = raw & 0xFF
        entries.append((idx, sub, bl))
    return cobid, ttype, entries

def store_parameters(node):
    """
    Try to persist parameters per CiA-301 0x1010:01 ("save").
    Strategy:
      1) If 0x1010 is in OD, write via OD wrapper.
      2) Else try raw SDO download to 0x1010:01 with ASCII 'save'.
      3) If both fail, print a clear message and continue (volatile config).
    """
    from canopen.sdo.exceptions import SdoAbortedError

    try:
        _ = node.sdo[0x1010]
        try:
            node.sdo[0x1010][1].raw = 0x65766173  # 'save' as UNSIGNED32
            print('Store request sent via OD to 0x1010:01 ("save").')
            return
        except SdoAbortedError as e:
            print(f'OD store failed (0x1010:01), SDO abort 0x{int(e.args[0]):08X}')
    except KeyError:
        pass

    try:
        node.sdo.download(0x1010, 0x01, b"save")
        print('Store request sent via RAW SDO to 0x1010:01 ("save").')
        return
    except SdoAbortedError as e:
        code = int(e.args[0])
        print(f'RAW SDO store failed, SDO abort 0x{code:08X} (device/firmware may not support 0x1010).')

    print("Persistence not available via 0x1010 on this unit/EDS. "
          "PDO mapping likely volatile; auto-remap at startup or use vendor NVS/USB config if supported.")

def main():
    ap = argparse.ArgumentParser(description="Remap RPDO1 (0x20A) and RPDO2 (0x30A) on a Delta-Q CANopen charger.")
    ap.add_argument("--channel", default="can0", help="CAN interface channel (default: can0)")
    ap.add_argument("--bustype", default="socketcan", help="python-can bustype (default: socketcan)")
    ap.add_argument("--node-id", type=lambda x: int(x, 0), default=0x0A, help="Charger node ID (default: 0x0A)")
    ap.add_argument("--eds", default="deltaq.eds", help="Path to EDS file")
    ap.add_argument("--save", action="store_true", help="After remapping, attempt to save parameters via 0x1010:01.")
    args = ap.parse_args()

    # COB-IDs from node ID (0x200+ID = 0x20A, 0x300+ID = 0x30A for ID=0x0A)
    cobid_rpdo1 = 0x200 + args.node_id
    cobid_rpdo2 = 0x300 + args.node_id

    net = canopen.Network()
    net.connect(channel=args.channel, bustype=args.bustype)

    node = canopen.RemoteNode(args.node_id, args.eds)
    net.add_node(node)

    # Move to PRE-OP for safe reconfiguration
    try:
        node.nmt.state = "PRE-OPERATIONAL"
    except Exception:
        pass
    time.sleep(0.1)

    try:
        # RPDO1 -> 0x20A
        remap_rpdo(node, rpdo_number=0, cobid=cobid_rpdo1,
                   tx_type=TRANSMISSION_TYPE, entries=RPDO1_ENTRIES)

        # RPDO2 -> 0x30A
        remap_rpdo(node, rpdo_number=1, cobid=cobid_rpdo2,
                   tx_type=TRANSMISSION_TYPE, entries=RPDO2_ENTRIES)

        # Verify
        cob1, tt1, m1 = verify_rpdo(node, 0)
        cob2, tt2, m2 = verify_rpdo(node, 1)
        print(f"RPDO1: COB-ID=0x{cob1:08X} (enabled={0==(cob1>>31)}), TT=0x{tt1:02X}, map={m1}")
        print(f"RPDO2: COB-ID=0x{cob2:08X} (enabled={0==(cob2>>31)}), TT=0x{tt2:02X}, map={m2}")

        if args.save:
            store_parameters(node)

    except SdoAbortedError as e:
        code = int(e.args[0])
        print(f"SDO abort 0x{code:08X}: {abort_meaning(code)}")
        raise
    finally:
        # Back to OPERATIONAL
        try:
            node.nmt.state = "OPERATIONAL"
        except Exception:
            pass

    # net.disconnect()  # Uncomment if you want to drop the bus at the end

if __name__ == "__main__":
    main()
