#!/usr/bin/env python3
import sys
import canopen
from canopen.objectdictionary import Variable, Record, Array

# ----- CONFIG -----
CHANNEL   = "can0"
BUSTYPE   = "socketcan"
NODE_ID   = 0x0A          # <-- charger node-id (change if needed)
EDS_PATH  = "deltaq.eds"  # <-- path to your EDS

# ----- Helpers -----
def fmt_sw_string(major: int, minor: int, patch: int, variant: int) -> str:
    """Format as '001.008.00000#00008'."""
    return f"{major:03}.{minor:03}.{patch:05}#{variant:05}"

def try_read_visible_string(node, index):
    try:
        raw = node.sdo[index].raw
        if isinstance(raw, (bytes, bytearray)):
            return bytes(raw).decode(errors="ignore").strip("\x00").strip()
        if isinstance(raw, str):
            return raw.strip()
    except KeyError:
        pass
    return None

def search_version_record(node):
    """
    Find a Record/Array that looks like a version schema (subindexes 1..4 numeric)
    and return a formatted string, or None.
    """
    od = node.object_dictionary
    candidates = []
    for obj in od.values():
        name = getattr(obj, "name", "") or ""
        if "version" in name.lower():
            candidates.append(obj)

    for obj in candidates:
        # Prefer Records with at least 4 subentries
        if isinstance(obj, (Record, Array)):
            try:
                maj = int(node.sdo[obj.index][1].raw)
                minr = int(node.sdo[obj.index][2].raw)
                patch = int(node.sdo[obj.index][3].raw)
                var  = int(node.sdo[obj.index][4].raw)
                return fmt_sw_string(maj, minr, patch, var), obj.index
            except Exception:
                continue
    return None, None

def read_manufacturer_software_version(node):
    """
    Strategy:
      1) 0x100A (Manufacturer software version) if present (string)
      2) Any Record/Array with 'version' in the name and 4 numeric subindices
         (major/minor/patch/variant) → formatted string
    """
    # 1) 0x100A (VISIBLE_STRING) is the CANopen standard place
    s = try_read_visible_string(node, 0x100A)
    if s:
        # Some devices already return the exact desired string, otherwise fall through to format
        return s, 0x100A

    # 2) Search for a numeric record that matches Major/Minor/Patch/Variant
    formatted, idx = search_version_record(node)
    if formatted:
        return formatted, idx

    raise KeyError("Manufacturer software version not found in EDS/SDO objects (tried 0x100A and searched candidates).")

def decode_active_algo(val: int):
    """
    0x2241 (Active Algo Request), UNSIGNED32.
      - On read:
          0xFFFFFFFF  → no active algorithm set
          else: low 16 bits = algo ID
                upper 16 bits: reserved (spec says reserved on read), but some FW may echo flags:
                   bit15 permanence (1=permanent), bit14 AC-loss reset enable (1=will reset)
    """
    if val == 0xFFFFFFFF:
        return {
            "present": False,
            "raw": val,
            "algo_id": None,
            "flags": None,
            "permanent": None,
            "ac_loss_resets": None,
        }
    algo_id = val & 0xFFFF
    flags = (val >> 16) & 0xFFFF
    permanent = bool((flags >> 15) & 1)
    ac_loss_resets = bool((flags >> 14) & 1)
    return {
        "present": True,
        "raw": val,
        "algo_id": algo_id,
        "flags": flags,
        "permanent": permanent,
        "ac_loss_resets": ac_loss_resets,
    }

def list_installed_algorithms(node):
    """
    Uses 0x2242/0x2243/0x2244 to enumerate algorithms:
      0x2242 (u8)  → installed count
      0x2243 (u8)  → set 'index' for subsequent 0x2244 read
      0x2244 (u32) → (Major<<24) | (Minor<<16) | (Id)
    Returns list of dicts: [{'index': i, 'id': id, 'major': maj, 'minor': min}, ...]
    """
    out = []
    try:
        count = int(node.sdo[0x2242].raw)
    except KeyError:
        return out  # not supported on this device/EDS
    except Exception as e:
        print("Failed to read Installed Algo Count (0x2242):", e)
        return out

    for i in range(count):
        try:
            node.sdo[0x2243].raw = i  # set index
            val = int(node.sdo[0x2244].raw)
        except Exception:
            continue
        if val == 0:
            continue
        major = (val >> 24) & 0xFF
        minor = (val >> 16) & 0xFF
        algo_id = val & 0xFFFF
        out.append({"index": i, "id": algo_id, "major": major, "minor": minor, "raw": val})
    return out

def main():
    # Connect
    net = canopen.Network()
    net.connect(channel=CHANNEL, bustype=BUSTYPE)
    node = canopen.RemoteNode(NODE_ID, EDS_PATH)
    net.add_node(node)

    # (Optional) ensure SDO works in current state
    try:
        node.nmt.state = "PRE-OPERATIONAL"
    except Exception:
        pass

    try:
        # --- Manufacturer Software Version ---
        sw_str, src_idx = read_manufacturer_software_version(node)
        print(f"Software Version [{hex(src_idx)}]: {sw_str}")

        # If the string isn't already in the '001.008.00000#00008' shape, try to parse & reformat
        if "#" not in sw_str and sw_str.count(".") == 2:
            # Attempt to parse like "1.8.0.8" → "001.008.00000#00008"
            parts = sw_str.replace("#", ".").split(".")
            if len(parts) >= 4:
                maj, minr, patch, variant = (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))
                print("Software Version (normalized):", fmt_sw_string(maj, minr, patch, variant))

        # --- Active Algo Request (0x2241) ---
        try:
            active_val = int(node.sdo[0x2241].raw)
            decoded = decode_active_algo(active_val)
            if not decoded["present"]:
                print("Active Algo (0x2241): none set (0xFFFFFFFF)")
            else:
                print(
                    f"Active Algo (0x2241): ID={decoded['algo_id']} "
                    f"flags=0x{decoded['flags']:04X} "
                    f"permanent={'yes' if decoded['permanent'] else 'no'} "
                    f"AC-loss resets={'yes' if decoded['ac_loss_resets'] else 'no'}"
                )
        except KeyError:
            print("Active Algo (0x2241) not present in EDS.")
        except Exception as e:
            print("Failed to read Active Algo (0x2241):", e)

        # --- Enumerate installed algorithms (optional) ---
        algos = list_installed_algorithms(node)
        if algos:
            print(f"Installed algorithms (count={len(algos)}):")
            for a in algos:
                print(f"  index={a['index']:3d}  ID={a['id']:5d}  version={a['major']}.{a['minor']:02d}  raw=0x{a['raw']:08X}")

    finally:
        net.disconnect()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
