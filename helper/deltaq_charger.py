import can
import cantools
import threading
import time
import signal
import sys
import argparse

class ChargerController(threading.Thread):
    def __init__(self, can_interface="can0", volts=0.0, amps=0.0, temperature=20.0, soc=50,
                 send_interval=1.0, dbc_path="/home/nmc220/honda-cb550/org_delta_q.dbc", read_only=False, node_id=1):
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

        # Messages from DBC
        self.rpdo1 = self.dbc.get_message_by_name("DeltaQ_RPDO1_0x20a")

        # Track NMT state transition
        self.nmt_started = False

    # --------------------------
    # CAN frame builders
    # --------------------------
    def send_heartbeat(self):
        msg = can.Message(arbitration_id=0x701,
                          data=[0x05],
                          is_extended_id=False)
        self.bus.send(msg)

    def send_battery_status(self, voltage, current, ready=True):
        data = self.rpdo1.encode({
            "Voltage_Request": voltage,
            "Charge_Current_Request": current,
            "Battery_Status": 1 if ready else 0,
            "Battery_SOC": self.soc,
            "Batt_Charge_Cycle_Type": 1
        })
        msg = can.Message(arbitration_id=self.rpdo1.frame_id,
                          data=data,
                          is_extended_id=False)
        self.bus.send(msg)

    def send_nmt_start(self):
        msg = can.Message(arbitration_id=0x000,
                          data=[0x01, self.node_id],
                          is_extended_id=False)
        self.bus.send(msg)
        print(f"Sent NMT Start Remote Node to node {self.node_id}")

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