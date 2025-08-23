import helper.decode_ttl as ttl
import helper.deltaq_charger as deltaq
import helper.charging_profile as charger

import serial
import sys
import time


def main():
    ser = serial.Serial("/dev/serial0", 115200, timeout=1)
    charging_control = charger.charging_control()
    dq_charger = deltaq.ChargerController(can_interface="can0",
                                volts=0,
                                amps=0,
                                temperature=0,
                                soc=0)

    while True:
        line = None
        if ser.in_waiting > 0:
            # Read a line (up to newline '\n')
            line = ser.readline().decode('utf-8').strip()
        if line is None:
            continue

        decoded_data = ttl.convert_full_msg(line)

        pack_soc = decoded_data["soc"]
        pack_voltage = decoded_data["pack_voltage"]
        pack_current = decoded_data["pack_current"]

        if pack_soc > .95:
            charging_control.stop_charging() #don't do charging
            dq_charger.stop()
            print("SOC greater than desired amount, stopping charging")
            sys.exit()
        else:
            if not charging_control.charging:
                charging_control.start_charging(pack_voltage, pack_current)
                dq_charger.start()

        request_current, request_voltage = charging_control.run_update(pack_voltage, pack_current)

        output = f"""
----------- {time.strftime('%Y-%m-%d %H:%M:%S')} -----------
Pack Voltage:           {pack_voltage:.2f} V
Pack Current:           {pack_current:.2f} A
Charging Power:         {pack_voltage * pack_current:.2f} W
Requested Charge Voltage: {request_voltage:.2f} V
Requested Charge Current: {request_current:.2f} A
===============================================
"""

        print(output)

        dq_charger.update(volts=request_voltage, amps=request_current, temperature=25.0, soc=pack_soc)
        

if __name__ == "__main__":
    main()