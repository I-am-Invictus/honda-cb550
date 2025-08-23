def convert_msg_to_hex_array(raw_string):
    hex_str = hex_str.replace(' ', '')
    # Convert every two hex characters to an integer
    return [int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2)]

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


# add in temps, etc


def convert_full_msg(raw_string):
    hex_array = convert_full_msg(raw_string)
    pack_voltage = decode_pack_voltage(hex_array)
    pack_current = decode_pack_current(hex_array)
    soc = decode_soc(hex_array)

    return {"pack_voltage": pack_voltage, "pack_current":pack_current, "soc": soc}
