import helper.decode_ttl as ttl
import helper.deltaq_charger as deltaq
import helper.charging_profile as charger

import serial
import sys
import time
import json


def main():
    # RS-485 Serial connection to BMS
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=19200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1  # seconds
    )

    # Charging controller and Delta-Q charger init
    charging_control = charger.charging_control()

    # 6-byte BMS query command
    bms_request = bytes.fromhex("5A5A00000000")

    while True:
        # Send query
        ser.write(bms_request)
        time.sleep(0.3)  # Give BMS time to respond

        # Read response
        response = ser.read(140)

        if len(response) != 140:
            print(f"Expected 140 bytes, got {len(response)}. Retrying...")
            continue
        else:
            response = list(response)

        try:
            # Decode data (assuming TTL decoding works with RS-485 frame)
            decoded_data = ttl.convert_full_msg(response)

            pack_soc = decoded_data["soc"]
            pack_voltage = decoded_data["pack_voltage"]
            pack_current = decoded_data["pack_current"]
            print(json.dumps(decoded_data, indent=2))

            if pack_soc > 95:
                charging_control.stop_charging()
                #dq_charger.stop()
                #print(f"SOC greater than desired amount ({pack_soc}%), stopping charging")
                sys.exit()
            else:
                if not charging_control.charging:
                    #charging_control.start_charging(pack_voltage, pack_current)
                    #dq_charger.start()
                    pass

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

            #dq_charger.update(volts=request_voltage, amps=request_current, temperature=25.0, soc=pack_soc)

        except Exception as e:
            print(f"Error decoding BMS data: {e}")
            continue


if __name__ == "__main__":
    main()
