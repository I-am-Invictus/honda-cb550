"""
Battery module simulator for Delta-Q charger bring-up
- Heartbeat producer (you = battery node)
- SDO server for a few key objects (device type, heartbeat time, TPDO COB-IDs)
- NMT command to charger (start/operational)
- Bring-up handshake: battery status + initial voltage/current requests
- RPDO1/2 periodic sender (mirrors setpoints)

Requires: python-can, cantools
"""

import cantools
import can
import threading
import time
import struct

# Load your DeltaQ dbc file (adjust path)
dbc = cantools.database.load_file("/home/nmc220/honda-cb550/docs/delta_q.dbc")

# --------------------- CONFIG ---------------------
BITRATE = 500000
BUSTYPE = "socketcan"
CHANNEL = "can0"

BATTERY_NODE_ID  = 0x01
CHARGER_NODE_ID  = 0x0A

HEARTBEAT_PERIOD_S = 0.1
# --------------------------------------------------

# COB-IDs used by CANopen:
NMT_ID               = 0x000
HEARTBEAT_BASE       = 0x700
SDO_CLIENT_TO_SERVER = 0x600
SDO_SERVER_TO_CLIENT = 0x580

def u8(v):  return struct.pack("<B", v)
def u32(v): return struct.pack("<I", v)
# --- clamp helpers ---
def clamp(val, lo, hi): return max(lo, min(hi, val))
def u16(val):  # little-endian unsigned
    return val.to_bytes(2, byteorder="little", signed=False)
def i16(val):  # little-endian signed
    return val.to_bytes(2, byteorder="little", signed=True)

TPDO1_COBID = 0x180 + BATTERY_NODE_ID
TPDO2_COBID = 0x280 + BATTERY_NODE_ID
TPDO3_COBID = 0x380 + BATTERY_NODE_ID

OD = {
    (0x1000, 0x00): u32(0x00000000),
    (0x1017, 0x00): u16(int(HEARTBEAT_PERIOD_S * 1000)),

    (0x1800, 0x01): u32(TPDO1_COBID),
    (0x1800, 0x02): u8(0xFF),
    (0x1800, 0x05): u16(100),

    (0x1801, 0x01): u32(TPDO2_COBID),
    (0x1801, 0x02): u8(0xFF),
    (0x1801, 0x05): u16(100),

    (0x1802, 0x01): u32(TPDO3_COBID),
    (0x1802, 0x02): u8(0xFF),
    (0x1802, 0x05): u16(100),

    (0x1A00, 0x00): u8(0),
    (0x1A01, 0x00): u8(0),
    (0x1A02, 0x00): u8(0),
}

def make_bus():
    return can.interface.Bus(bustype=BUSTYPE, channel=CHANNEL, bitrate=BITRATE)

# --------------------- HEARTBEAT ---------------------
def start_heartbeat(bus: can.BusABC, period=0.05):
    """
    Start heartbeat producer with python-can's send_periodic.
    Returns the task object so you can stop() later.
    """
    cob_id = HEARTBEAT_BASE + BATTERY_NODE_ID
    state_byte = 0x05  # Operational

    msg = can.Message(arbitration_id=cob_id,
                      is_extended_id=False,
                      data=[state_byte])

    task = bus.send_periodic(msg, period)
    print(f"Started heartbeat @ {period*1000:.0f}ms, COB-ID=0x{cob_id:03X}")
    return task

# ----------------------- NMT -------------------------
def nmt_start_charger(bus: can.BusABC):
    data = bytes([0x01, CHARGER_NODE_ID])   # 0x01 = Start
    msg  = can.Message(arbitration_id=NMT_ID, is_extended_id=False, data=data)
    bus.send(msg)

# -------------------- Charger Monitor ----------------
def monitor_charger_state(bus: can.BusABC, stop_ev: threading.Event):
    charger_cobid = HEARTBEAT_BASE + CHARGER_NODE_ID
    last_state = None

    while not stop_ev.is_set():
        msg = bus.recv(timeout=0.2)
        if msg is None:
            continue

        try:
            decoded = dbc.decode_message(msg.arbitration_id, msg.data)
            print(f"Decoded [{hex(msg.arbitration_id)}]: {decoded}")
        except Exception:
            print(f"Raw [{hex(msg.arbitration_id)}]: {msg.data.hex()}")

        if msg.arbitration_id == charger_cobid and len(msg.data) >= 1:
            state = msg.data[0]
            if state != last_state:
                state_txt = {0x00: "Bootup", 0x04: "Stopped", 0x05: "Operational", 0x7F: "Pre-op"}.get(state, "Unknown")
                print(f"Charger heartbeat → state=0x{state:02X} ({state_txt})")
                last_state = state

# ----------------- Bring-up Handshake ----------------
def sdo_download(bus: can.BusABC, index: int, subindex: int, value: int, size: int):
    tx_cobid = SDO_CLIENT_TO_SERVER + CHARGER_NODE_ID   # 0x600 + node-id

    # Command specifier based on size
    cs = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]

    # Build payload
    data = bytes([cs, index & 0xFF, (index >> 8) & 0xFF, subindex]) \
           + value.to_bytes(size, byteorder="little") \
           + b"\x00" * (4 - size)

    # Send request
    msg = can.Message(arbitration_id=tx_cobid, is_extended_id=False, data=data)
    bus.send(msg)

    print(f"→ Sent SDO download 0x{index:04X}/{subindex:02X} = {value}")


def bringup_sequence(bus: can.BusABC, vreq: float, ireq: float):
    # Step 2a: status=0
    # sdo_download(bus, 0x6000, 0x00, 0, 1)
    # time.sleep(1.0)
    # Step 2b: status=1
    sdo_download(bus, 0x6000, 0x00, 1, 1)
    time.sleep(1.0)
    # Step 3: seed voltage/current requests
    vreq_raw = int(vreq / 0.00390625) & 0xFFFFFFFF  # 32-bit mask
    ireq_raw = int(ireq / 0.0625) & 0xFFFF
    sdo_download(bus, 0x2271, 0x00, vreq_raw, 4)
    time.sleep(1)
    sdo_download(bus, 0x6070, 0x00, ireq_raw, 2)
    time.sleep(1)


def pad8(data: bytes) -> bytes:
    """Pad CAN payload to 8 bytes with zeros (if shorter)."""
    return data + bytes(8 - len(data))

#------------- RPDO sender (matches updated DBC) ---------------------------
def i16(v): return struct.pack("<h", v)  # add near your other pack helpers

#------------- RPDO sender (matches updated DBC) ---------------------------
def send_rpdos(bus: can.BusABC, stop_ev: threading.Event,
               soc=76, vreq=82.0, ireq=5.0, temperature=30.0,
               cycle_type=0, batt_status=1, charging_current=10.0, vbat=82.0):
    """
    Periodically send:
      RPDO1 @ 0x20A:
        b0: 0x2FFA:01 (u8, reserved)            -> 0x00 (or rolling counter below)
        b1: 0x6081:00 Battery_SOC (u8)          -> clamp 0..100
        b2: 0x4201:00 Charge Cycle Type (u8)    -> as given
        b3-4: 0x2276:00 Voltage_Request (u16 LE)-> raw = round(V * 256)
        b5-6: 0x6070:00 Battery_Current (u16 LE)-> raw = round(A * 16)
        b7: 0x6000:00 Battery_Status (u8)       -> as given

      RPDO2 @ 0x30A:
        b0: 0x2FFA:01 (u8, reserved)            -> 0x00 (or rolling counter)
        b1: 0x2FFA:01 (u8, reserved)            -> 0x00
        b2-3: 0x2002:00 Charging Current (u16)  -> raw = round(A * 256)
        b4-5: 0x2101:00 Battery Voltage (u16)   -> raw = round(V * 256)
        b6-7: 0x6010:00 Battery Temperature(i16)-> raw = round(°C)  (1 °C/LSB)
    """
    RPDO1_COBID = 0x20A  # 522
    RPDO2_COBID = 0x30A  # 778

    # 10 Hz to match 0x180x/0x1A0x period 100ms in your OD above
    period = .15
    next_time = time.monotonic()

    # Simple rolling counter for the 0x2FFA:01 bytes (optional; set to 0x00 if undesired)
    counter = 0

    while not stop_ev.is_set():
        # --- Inputs → raw encodings ---
        soc_u8         = clamp(int(round(soc)), 0, 100)             # %
        cycle_u8       = clamp(int(round(cycle_type)), 0, 255)
        if counter == 0:
            batt_status_u8 = clamp(int(round(0)), 0, 255)
        else:
            batt_status_u8 = clamp(int(round(1)), 0, 255)

        vreq_u16_256   = clamp(int(round(vreq * 256.0)), 0, 0xFFFF)   # V * 256
        ireq_u16_16    = clamp(int(round(ireq * 16.0)),  0, 0xFFFF)   # A * 16

        charge_A_u16_256 = clamp(int(round(charging_current * 16.0)), 0, 0xFFFF)  # A * 16
        bat_volt_u16_256 = clamp(int(round(vbat * 256.0)), 0, 0xFFFF)              # V * 256
        temp_i16 = int(round((temperature + 40.0) / 0.125))  # scale + offset

        if temp_i16 < -32768: temp_i16 = -32768
        if temp_i16 >  32767: temp_i16 =  32767

        # --- RPDO1 payload (0x20A) ---
        # Byte0: reserved/counter (DBC doesn’t map it)
        b0 = 0x00
        b1 = soc_u8
        b2 = cycle_u8
        vreq_bytes = u16(vreq_u16_256)
        ireq_bytes = u16(ireq_u16_16)
        b7 = batt_status_u8

        rpdo1_data = bytes([b0, b1, b2]) + vreq_bytes + ireq_bytes + bytes([b7])

        # --- RPDO2 payload (0x30A) ---
        rb0 = 0x00
        rb1 = 0x00
        cur_bytes = u16(charge_A_u16_256)
        vol_bytes = u16(bat_volt_u16_256)
        tmp_bytes = i16(temp_i16)

        rpdo2_data = bytes([rb0, rb1]) + cur_bytes + vol_bytes + tmp_bytes

        # --- Transmit ---
        try:
            msg1 = can.Message(arbitration_id=RPDO1_COBID, is_extended_id=False, data=rpdo1_data)
            msg2 = can.Message(arbitration_id=RPDO2_COBID, is_extended_id=False, data=rpdo2_data)
            bus.send(msg1)
            # bus.send(msg2)
            print(
                f"TX RPDO1 0x{RPDO1_COBID:03X} [{rpdo1_data.hex()}]  "
                f"(SOC={soc_u8}%, Cycle={cycle_u8}, Vreq={vreq}V→0x{vreq_u16_256:04X}, "
                f"Ireq={ireq}A→0x{ireq_u16_16:04X}, Status=0x{batt_status_u8:02X})"
            )
            # print(
            #     f"TX RPDO2 0x{RPDO2_COBID:03X} [{rpdo2_data.hex()}]  "
            #     f"(Ibat={charging_current}A→0x{charge_A_u16_256:04X}, "
            #     f"Vbat={vbat}V→0x{bat_volt_u16_256:04X}, T={temp_i16/8}°C)"
            # )
        except can.CanError:
            # Keep running even if a frame drops
            pass

        counter = (counter + 1) & 0xFF
        next_time += period
        time.sleep(max(0, next_time - time.monotonic()))





# -------------------- MAIN --------------------------
def main():
    bus = make_bus()
    stop_ev = threading.Event()

    hb_task = start_heartbeat(bus, period=0.1)
    monitor_thr = threading.Thread(target=monitor_charger_state, args=(bus, stop_ev), daemon=True)
    rpdo_thr = threading.Thread(target=send_rpdos, args=(bus, stop_ev), daemon=True)

    
    monitor_thr.start()

    print("Sending NMT Start to charger...")
    nmt_start_charger(bus)

    # start the heartbeat after the charger starts
    time.sleep(1.0) # Give it a second

    # # Run bring-up handshake once
    #bringup_sequence(bus, vreq=84.0, ireq=5.0)

    rpdo_thr.start()
    print("Running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        hb_task.stop()
        monitor_thr.join(timeout=1.0)
        rpdo_thr.join(timeout=1.0)
        bus.shutdown()

if __name__ == "__main__":
    main()
