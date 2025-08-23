import can
import argparse
import time
import signal
import sys

def build_frame_20x30a(volts, amps, temperature):
    """
    Mock encoder for DeltaQRpdo20x30a (customize based on actual message format).
    """
    # Placeholder for 8-byte CAN frame data:
    data = bytearray(8)
    
    # Example encoding: pack voltage/current/temperature as 16-bit ints
    voltage_raw = int(volts * 10)    # assuming 0.1V/bit
    current_raw = int(amps * 10)     # assuming 0.1A/bit
    temp_raw = int(temperature)      # 1°C/bit

    data[0:2] = voltage_raw.to_bytes(2, 'little')
    data[2:4] = current_raw.to_bytes(2, 'little')
    data[4:6] = temp_raw.to_bytes(2, 'little')
    data[6:] = b'\x00\x00'  # reserved

    return can.Message(arbitration_id=0x20, data=data, is_extended_id=False)

def build_frame_10x20a(volts, amps, soc):
    """
    Mock encoder for DeltaQRpdo10x20a (customize based on actual message format).
    """
    data = bytearray(8)

    voltage_raw = int(volts * 10)    # assuming 0.1V/bit
    current_raw = int(amps * 10)     # assuming 0.1A/bit
    data[0] = soc  # state of charge (0-100%)

    data[1:3] = voltage_raw.to_bytes(2, 'little')
    data[3:5] = current_raw.to_bytes(2, 'little')
    data[5:] = b'\x00\x00\x00'  # status + reserved

    return can.Message(arbitration_id=0x10, data=data, is_extended_id=False)

def send_commands(bus, volts, amps, temperature, soc):
    msg1 = build_frame_20x30a(volts, amps, temperature)
    msg2 = build_frame_10x20a(volts, amps, soc)
    bus.send(msg1)
    bus.send(msg2)

def main():
    parser = argparse.ArgumentParser(description="Send voltage/current command to DeltaQ via CAN")
    parser.add_argument("-v", "--volts", type=float, required=True)
    parser.add_argument("-a", "--amps", type=float, required=True)
    parser.add_argument("-t", "--temperature", type=float, default=20.0)
    parser.add_argument("-s", "--soc", type=int, default=50)
    parser.add_argument("-c", "--can_interface", default="can0")

    args = parser.parse_args()

    bus = can.interface.Bus(channel=args.can_interface, bustype='socketcan')

    def handle_exit(sig, frame):
        print("Stopping and sending 0A/0V command to shut down charger.")
        send_commands(bus, 0.0, 0.0, args.temperature, args.soc)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)

    print(f"Sending voltage={args.volts}V current={args.amps}A temp={args.temperature}°C soc={args.soc}%")

    while True:
        send_commands(bus, args.volts, args.amps, args.temperature, args.soc)
        time.sleep(1) #need to send commands every 1 second.

if __name__ == "__main__":
    main()
