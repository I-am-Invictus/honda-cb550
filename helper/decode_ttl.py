def convert_msg_to_hex_array(byte_list):
    return byte_list

def decode_pack_voltage(hex_array):

    data4 = hex_array[4]
    data5 = hex_array[5]

    raw_value = (data4 << 8) | data5  # Same as data4 * 256 + data5
    voltage = raw_value / 10.0 #output to volts
    return voltage

def decode_individual_cells(hex_array, num_cells=20):
    cell_voltages = []
    for i in range(num_cells):
        high_byte = hex_array[i*2+6]
        low_byte = hex_array[i*2+7]

        raw_value = (high_byte << 8) | low_byte
        voltage = raw_value / 1000.0 #output to volts

        cell_voltages.append(voltage)
    
    return cell_voltages

def decode_pack_current(hex_array):
    high_byte = hex_array[72]
    low_byte = hex_array[73]

    raw_value = (high_byte << 8) | low_byte  # Same as data4 * 256 + data5
    current = raw_value / 10.0 #output to amps
    return current

def decode_soc(hex_array):
    soc_byte = hex_array[74] # raw %

    return soc_byte

def decode_MOS_temp(hex_array):
    high_byte = hex_array[91]
    low_byte = hex_array[92]

    raw_value = (high_byte << 8) | low_byte  # Same as data4 * 256 + data5
    temp = raw_value
    return temp

def decode_balance_temp(hex_array):
    high_byte = hex_array[93]
    low_byte = hex_array[94]

    raw_value = (high_byte << 8) | low_byte  # Same as data4 * 256 + data5
    temp = raw_value
    return temp

def decode_external_temps(hex_array):
    external_temps = []
    for i in range(4):
        high_byte = hex_array[i*2+95]
        low_byte = hex_array[i*2+96]

        raw_value = (high_byte << 8) | low_byte
        temp = raw_value

        external_temps.append(temp)
    
    return external_temps

def decode_physical_capacity(hex_array):

    #byte_array = [hex_array[78], hex_array[77], hex_array[76], hex_array[75]]
    byte_array = hex_array[75:79][::-1]

    byte0, byte1, byte2, byte3 = byte_array  # Little-endian order

    result = byte3 * 256**3 + byte2 * 256**2 + byte1 * 256 + byte0
    capacity_ah = result * 0.000001  # Apply resolution

    return capacity_ah

def decode_remaining_capacity(hex_array):

    #byte_array = [hex_array[78], hex_array[77], hex_array[76], hex_array[75]]
    byte_array = hex_array[79:83][::-1]

    byte0, byte1, byte2, byte3 = byte_array  # Little-endian order

    result = byte3 * 256**3 + byte2 * 256**2 + byte1 * 256 + byte0
    capacity_ah = result * 0.000001  # Apply resolution

    return capacity_ah

def decode_cyclic_capcity(hex_array):

    #byte_array = [hex_array[78], hex_array[77], hex_array[76], hex_array[75]]
    byte_array = hex_array[83:87][::-1]

    byte0, byte1, byte2, byte3 = byte_array  # Little-endian order

    result = byte3 * 256**3 + byte2 * 256**2 + byte1 * 256 + byte0
    capacity_ah = result * 0.000001  # Apply resolution

    return capacity_ah

def decode_charging_mos_status(hex_array):
    status = hex_array[103]
    charging_mos_status = {
        0: "Close",
        1: "Open",
        2: "Overvoltage of the single cell",
        3: "Over current",
        4: "Secondary overcurrent",
        5: "The total voltage is overvoltage",
        6: "Battery over-temperature",
        7: "Power over-temperature",
        8: "Current anomaly",
        9: "Balance line Disconnect",
        10: "mainboard over-temperature",
        11: "",
        12: "Failed to open",
        13: "Charging MOS Error",
        14: "waiting",
        15: "Manual close",
        16: "Secondary overvoltage",
        17: "Low-temperature protection",
        18: "Differential voltage protection",
        19: "",
        20: "",
        21: "",
        22: "Total voltage single cell abnormality"
    }
    return charging_mos_status[status], status

def decode_discharge_mos_status(hex_array):
    status = hex_array[104]

    discharge_mos_state = {
        0: "Close",
        1: "Open",
        2: "Under-voltage of the single cell",
        3: "Over current",
        4: "Secondary overcurrent",
        5: "The total voltage is under-voltage",
        6: "Battery over-temperature",
        7: "Power over-temperature",
        8: "Current anomaly",
        9: "Balance line Disconnect",
        10: "mainboard over-temperature",
        11: "",
        12: "Short circuit protection",
        13: "Discharge MOS Error",
        14: "Failed to open",
        15: "Manual close",
        16: "Under-voltage at level 2",
        17: "Low-temperature protection",
        18: "Differential voltage protection",
        19: "",
        20: "",
        21: "",
        22: "Total voltage single cell abnormality"
    }
    return discharge_mos_state[status], status


def decode_balance_status(hex_array):
    status = hex_array[105]
    balance_state = {
        0: "Close",
        1: "Balance limit",
        2: "Differential voltage Balance",
        3: "Balance over-temperature",
        4: "Auto Balance",
        10: "mainboard over-temperature"
    }
    return balance_state[status], status

def decode_high_cell_voltage(hex_array):
    cell_number = hex_array[115]
    high_byte = hex_array[116]
    low_byte = hex_array[117]

    raw_value = (high_byte << 8) | low_byte
    voltage = raw_value / 1000.0 #output to volts

    return cell_number, voltage

def decode_low_cell_voltage(hex_array):
    cell_number = hex_array[118]
    high_byte = hex_array[119]
    low_byte = hex_array[120]

    raw_value = (high_byte << 8) | low_byte
    voltage = raw_value / 1000.0 #output to volts

    return cell_number, voltage


def convert_full_msg(byte_list):
    hex_array = byte_list

    pack_voltage = decode_pack_voltage(hex_array)
    pack_current = decode_pack_current(hex_array)
    soc = decode_soc(hex_array)
    individual_cells = decode_individual_cells(hex_array)
    mos_temp = decode_MOS_temp(hex_array)
    balance_temp = decode_balance_temp(hex_array)
    external_temps = decode_external_temps(hex_array)
    physical_capacity = decode_physical_capacity(hex_array)
    remaining_capacity = decode_remaining_capacity(hex_array)
    cyclic_capacity = decode_cyclic_capcity(hex_array)

    discharge_mos_status_txt, discharge_mos_status = decode_discharge_mos_status(hex_array)
    charge_most_status_txt, charge_most_status = decode_charging_mos_status(hex_array)
    balance_status_txt, balance_status = decode_balance_status(hex_array)

    high_cell_number, high_cell_voltage = decode_high_cell_voltage(hex_array)
    low_cell_number, low_cell_voltage = decode_low_cell_voltage(hex_array)

    return {
        "pack_voltage": pack_voltage,
        "pack_current": pack_current,
        "soc": soc,
        "individual_cells": individual_cells,
        "mos_temperature": mos_temp,
        "balance_temperature": balance_temp,
        "external_temperatures": external_temps,
        "physical_capacity_ah": physical_capacity,
        "remaining_capacity_ah": remaining_capacity,
        "cyclic_capacity_ah": cyclic_capacity,

        "discharge_mos_status": {
            "codes": discharge_mos_status,
            "descriptions": discharge_mos_status_txt
        },
        "charge_mos_status": {
            "codes": charge_most_status,
            "descriptions": charge_most_status_txt
        },
        "balance_status": {
            "codes": balance_status,
            "descriptions": balance_status_txt
        },

        "high_cell": {
            "cell_number": high_cell_number,
            "voltage": high_cell_voltage
        },
        "low_cell": {
            "cell_number": low_cell_number,
            "voltage": low_cell_voltage
        }
    }

