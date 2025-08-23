import can
import threading
import time
import signal
import sys

class ChargerController(threading.Thread):
    def __init__(self, can_interface="can0", volts=0.0, amps=0.0, temperature=20.0, soc=50, send_interval=1.0):
        super().__init__()
        self.daemon = True  # optional, thread exits when main program exits
        self.can_interface = can_interface
        self.volts = volts
        self.amps = amps
        self.temperature = temperature
        self.soc = soc
        self.send_interval = send_interval
        
        self._running = threading.Event()
        self._running.set()
        self._lock = threading.Lock()  # protect shared data

        self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

    def build_frame_20x30a(self, volts, amps, temperature):
        data = bytearray(8)
        voltage_raw = int(volts * 10)    # 0.1V/bit
        current_raw = int(amps * 10)     # 0.1A/bit
        temp_raw = int(temperature)      # 1°C/bit
        data[0:2] = voltage_raw.to_bytes(2, 'little')
        data[2:4] = current_raw.to_bytes(2, 'little')
        data[4:6] = temp_raw.to_bytes(2, 'little')
        data[6:] = b'\x00\x00'  # reserved
        return can.Message(arbitration_id=0x20, data=data, is_extended_id=False)

    def build_frame_10x20a(self, volts, amps, soc):
        data = bytearray(8)
        voltage_raw = int(volts * 10)    # 0.1V/bit
        current_raw = int(amps * 10)     # 0.1A/bit
        data[0] = soc  # state of charge (0-100%)
        data[1:3] = voltage_raw.to_bytes(2, 'little')
        data[3:5] = current_raw.to_bytes(2, 'little')
        data[5:] = b'\x00\x00\x00'  # status + reserved
        return can.Message(arbitration_id=0x10, data=data, is_extended_id=False)

    def send_commands(self):
        with self._lock:
            volts = self.volts
            amps = self.amps
            temperature = self.temperature
            soc = self.soc

        msg1 = self.build_frame_20x30a(volts, amps, temperature)
        msg2 = self.build_frame_10x20a(volts, amps, soc)
        self.bus.send(msg1)
        self.bus.send(msg2)

    def run(self):
        while self._running.is_set():
            try:
                self.send_commands()
            except can.CanError as e:
                print(f"CAN send error: {e}")
            time.sleep(self.send_interval)

    def stop(self):
        self._running.clear()
        # Send zero commands to safely stop charging
        try:
            self.volts = 0.0
            self.amps = 0.0
            self.send_commands()
        except can.CanError:
            pass
        self.bus.shutdown()

    def update(self, volts=None, amps=None, temperature=None, soc=None):
        with self._lock:
            if volts is not None:
                self.volts = volts
            if amps is not None:
                self.amps = amps
            if temperature is not None:
                self.temperature = temperature
            if soc is not None:
                self.soc = soc

# Example usage
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Threaded DeltaQ CAN charger controller")
    parser.add_argument("-v", "--volts", type=float, default=0.0)
    parser.add_argument("-a", "--amps", type=float, default=0.0)
    parser.add_argument("-t", "--temperature", type=float, default=20.0)
    parser.add_argument("-s", "--soc", type=int, default=50)
    parser.add_argument("-c", "--can_interface", default="can0")
    args = parser.parse_args()

    charger = ChargerController(can_interface=args.can_interface,
                                volts=args.volts,
                                amps=args.amps,
                                temperature=args.temperature,
                                soc=args.soc)
    charger.start()

    def handle_exit(sig, frame):
        print("\nStopping charger...")
        charger.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)

    print(f"Started charger thread with V={args.volts}A={args.amps}T={args.temperature}°C SOC={args.soc}%")
    print("Press Ctrl+C to stop.")

    # Example: dynamically update charging values every 10 seconds (demo)
    try:
        while True:
            time.sleep(10)
            # Example update, in real case update as needed
            charger.update(volts=48.0, amps=15.0, temperature=25.0, soc=60)
            print("Updated charger params.")
    except KeyboardInterrupt:
        handle_exit(None, None)