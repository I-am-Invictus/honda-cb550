import can
import cantools
import threading
import time
import signal
import sys
import argparse

# --- pack helpers using to_bytes ---
def u8(v: int) -> bytes:
    """Unsigned 8-bit, little-endian"""
    return v.to_bytes(1, byteorder="little", signed=False)

def u16(v: int) -> bytes:
    """Unsigned 16-bit, little-endian"""
    return v.to_bytes(2, byteorder="little", signed=False)

def i16(v: int) -> bytes:
    """Signed 16-bit, little-endian"""
    return v.to_bytes(2, byteorder="little", signed=True)

def u32(v: int) -> bytes:
    """Unsigned 32-bit, little-endian"""
    return v.to_bytes(4, byteorder="little", signed=False)

# --- clamp helper ---
def clamp(val: int | float, lo: int | float, hi: int | float):
    """Clamp val into [lo, hi]"""
    return max(lo, min(hi, val))

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

RPDO1_COBID = 0x20A  # 522


class ChargerController(threading.Thread):
    def __init__(self, can_interface="can0", volts=0.0, amps=0.0, temperature=20.0, soc=50,
                 send_interval=1.0, dbc_path="/home/nmc220/honda-cb550/docs/org_delta_q.dbc", read_only=False, node_id=1):
        super().__init__()
        self.daemon = True
        self.can_interface = can_interface
        self.volts = volts
        self.amps = amps
        self.temperature = temperature
        self.soc = soc
        self.send_interval = send_interval
        self.read_only = read_only
        self.node_id = node_id

        self._running = threading.Event()
        self._running.set()
        self._lock = threading.Lock()
        self._state_lock = threading.Lock()

        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

        # Load DBC
        self.dbc = cantools.database.load_file(dbc_path)
        self.current_state = {}

    # --------------------------
    # CAN frame builders
    # --------------------------

    def start_heartbeat(self, period=0.05):
        """
        Start heartbeat producer with python-can's send_periodic.
        Returns the task object so you can stop() later.
        """
        cob_id = HEARTBEAT_BASE + BATTERY_NODE_ID
        state_byte = 0x05  # Operational

        msg = can.Message(arbitration_id=cob_id,
                        is_extended_id=False,
                        data=[state_byte])

        self.hb_task = self.bus.send_periodic(msg, period)
        print(f"Started heartbeat @ {period*1000:.0f}ms, COB-ID=0x{cob_id:03X}")
    
    def stop_heartbeat(self):
        self.hb_task.stop()
        print("Stopping heartbeat message")

    def send_0x20a(self, soc=76, vreq=82.0, ireq=2.0, temperature=30.0,
               cycle_type=0):
        soc_u8         = clamp(int(round(soc)), 0, 100)             # %
        cycle_u8       = clamp(int(round(cycle_type)), 0, 255)
        if self.counter == 0:
            batt_status_u8 = clamp(int(round(0)), 0, 255)
        else:
            batt_status_u8 = clamp(int(round(1)), 0, 255)

        vreq_u16_256   = clamp(int(round(vreq * 256.0)), 0, 0xFFFF)   # V * 256
        ireq_u16_16    = clamp(int(round(ireq * 16.0)),  0, 0xFFFF)   # A * 16

        # --- RPDO1 payload (0x20A) ---
        # Byte0: reserved/counter (DBC doesn’t map it)
        b0 = 0x00
        b1 = soc_u8
        b2 = cycle_u8
        vreq_bytes = u16(vreq_u16_256)
        ireq_bytes = u16(ireq_u16_16)
        b7 = batt_status_u8

        rpdo1_data = bytes([b0, b1, b2]) + vreq_bytes + ireq_bytes + bytes([b7])

        msg1 = can.Message(arbitration_id=RPDO1_COBID, is_extended_id=False, data=rpdo1_data)
        self.bus.send(msg1)

        self.counter = self.counter + 1

    def nmt_start_charger(self):
        data = bytes([0x01, CHARGER_NODE_ID])   # 0x01 = Start
        msg  = can.Message(arbitration_id=NMT_ID, is_extended_id=False, data=data)
        self.bus.send(msg)

    # --------------------------
    # Thread loop
    # --------------------------
    def run(self):
        receiver_timeout = 0.1
        last_send_time = time.time()

        while self._running.is_set():
            current_time = time.time()

            # Periodic sends only after Operational
            if not self.read_only and self.nmt_started and (current_time - last_send_time >= self.send_interval):
                try:
                    with self._lock:
                        volts = self.volts
                        amps = self.amps
                    self.send_heartbeat()
                    self.send_battery_status(volts, amps, ready=True)
                    print("Sent heartbeat and battery status")
                except can.CanError as e:
                    print(f"CAN send error: {e}")
                last_send_time = current_time

            # Receive
            try:
                msg = self.bus.recv(timeout=receiver_timeout)
                if msg:
                    self.process_received_message(msg)
            except can.CanError as e:
                print(f"CAN receive error: {e}")

    def process_received_message(self, msg):
        try:
            decoded = self.dbc.decode_message(msg.arbitration_id, msg.data)
            with self._state_lock:
                self.current_state[msg.arbitration_id] = decoded
            print(f"Decoded [{hex(msg.arbitration_id)}]: {decoded}")

            # Detect charger heartbeat state
            if "Heartbeat" in decoded:
                if decoded["Heartbeat"] == "Pre-operational" and not self.nmt_started and not self.read_only:
                    print("Charger in Pre-operational → sending NMT Start")
                    self.send_nmt_start()
                    
                elif decoded["Heartbeat"] == "Operational":
                    self.nmt_started = True
                    print("Charger is now Operational")

        except (KeyError, ValueError):
            print(f"Unhandled: {msg.arbitration_id} | {msg.data}")

    def get_current_state(self):
        with self._state_lock:
            return self.current_state.copy()

    def update(self, volts=None, amps=None, soc=None):
        with self._lock:
            if volts is not None:
                self.volts = volts
            if amps is not None:
                self.amps = amps
            if soc is not None:
                self.soc = soc

    def stop(self):
        self._running.clear()
        if not self.read_only and self.nmt_started:
            try:
                self.send_battery_status(0, 0, ready=False)
            except can.CanError:
                pass
        self.stop_heartbeat()
        self.bus.shutdown()


# --------------------------
# Main entry
# --------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DeltaQ Charger Controller with DBC decoding")
    parser.add_argument("-v", "--volts", type=float, default=0.0)
    parser.add_argument("-a", "--amps", type=float, default=0.0)
    parser.add_argument("-s", "--soc", type=int, default=50)
    parser.add_argument("-c", "--can_interface", default="can0")
    parser.add_argument("-d", "--dbc", required=True, help="Path to DBC file")
    parser.add_argument("--read_only", action="store_true")
    parser.add_argument("--node_id", type=int, default=1, help="CANopen node ID of charger")
    args = parser.parse_args()

    charger = ChargerController(can_interface=args.can_interface,
                                volts=args.volts,
                                amps=args.amps,
                                soc=args.soc,
                                dbc_path=args.dbc,
                                read_only=args.read_only,
                                node_id=args.node_id)
    charger.start()

    def handle_exit(sig, frame):
        print("\nStopping charger...")
        charger.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)

    if args.read_only:
        print("Started in READ-ONLY mode. Listening only...")
    else:
        print(f"Started charger thread with V={args.volts} A={args.amps} SOC={args.soc}%")
        print("Waiting for Pre-operational heartbeat to send NMT Start...")

    try:
        while True:
            time.sleep(5)
            print("Current decoded state:", charger.get_current_state())
    except KeyboardInterrupt:
        handle_exit(None, None)