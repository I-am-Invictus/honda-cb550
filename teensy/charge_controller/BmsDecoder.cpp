#include "BmsDecoder.h"

// ---------------- Helper: 16-bit and 32-bit assembly ----------------
static inline uint16_t u16(const uint8_t *d) {
  return (uint16_t(d[0]) << 8) | d[1];
}

static inline uint32_t u32_be(const uint8_t *d) {
  return (uint32_t(d[0]) << 24) | (uint32_t(d[1]) << 16) |
         (uint32_t(d[2]) << 8) | d[3];
}

// ---------------- Decoders ----------------
BmsData decodeBmsMessage(const uint8_t *hex_array, size_t len) {
  BmsData out;

  if (len < 121) { // need enough bytes
    return out;
  }

  // Pack voltage = bytes [4],[5]
  out.pack_voltage_V = ((hex_array[4] << 8) | hex_array[5]) / 10.0f;

  // Pack current = bytes [72],[73]
  out.pack_current_A = ((hex_array[72] << 8) | hex_array[73]) / 10.0f;

  // SOC = byte [74]
  out.soc_pct = hex_array[74];

  // Cells = bytes [6..(6+num*2-1)]
  const int num_cells = 20;
  out.cell_voltages.resize(num_cells);
  for (int i = 0; i < num_cells; i++) {
    uint8_t hi = hex_array[i*2 + 6];
    uint8_t lo = hex_array[i*2 + 7];
    uint16_t raw = (hi << 8) | lo;
    out.cell_voltages[i] = raw / 1000.0f;
  }

  // MOS temp = [91,92]
  out.mos_temperature_C = ((hex_array[91] << 8) | hex_array[92]);

  // Balance temp = [93,94]
  out.balance_temperature_C = ((hex_array[93] << 8) | hex_array[94]);

  // External temps = 4x words from [95..102]
  out.external_temperatures.resize(4);
  for (int i = 0; i < 4; i++) {
    uint16_t raw = (hex_array[95 + i*2] << 8) | hex_array[96 + i*2];
    out.external_temperatures[i] = raw;
  }

  // Physical capacity = [75..78] (big endian 4B → Ah * 1e-6)
  {
    uint32_t raw = u32_be(&hex_array[75]);
    out.physical_capacity_Ah = raw * 1e-6f;
  }

  // Remaining capacity = [79..82]
  {
    uint32_t raw = u32_be(&hex_array[79]);
    out.remaining_capacity_Ah = raw * 1e-6f;
  }

  // Cyclic capacity = [83..86]
  {
    uint32_t raw = u32_be(&hex_array[83]);
    out.cyclic_capacity_Ah = raw * 1e-6f;
  }

  // Charging MOS status = [103]
  {
    uint8_t code = hex_array[103];
    out.charge_mos_status_code = code;
    switch(code) {
      case 0: out.charge_mos_status_txt="Close"; break;
      case 1: out.charge_mos_status_txt="Open"; break;
      case 2: out.charge_mos_status_txt="Overvoltage of the single cell"; break;
      case 3: out.charge_mos_status_txt="Over current"; break;
      case 13: out.charge_mos_status_txt="Charging MOS Error"; break;
      default: out.charge_mos_status_txt="Unknown"; break;
    }
  }

  // Discharge MOS status = [104]
  {
    uint8_t code = hex_array[104];
    out.discharge_mos_status_code = code;
    switch(code) {
      case 0: out.discharge_mos_status_txt="Close"; break;
      case 1: out.discharge_mos_status_txt="Open"; break;
      case 2: out.discharge_mos_status_txt="Under-voltage of the single cell"; break;
      case 3: out.discharge_mos_status_txt="Over current"; break;
      case 13: out.discharge_mos_status_txt="Discharge MOS Error"; break;
      default: out.discharge_mos_status_txt="Unknown"; break;
    }
  }

  // Balance status = [105]
  {
    uint8_t code = hex_array[105];
    out.balance_status_code = code;
    switch(code) {
      case 0: out.balance_status_txt="Close"; break;
      case 1: out.balance_status_txt="Balance limit"; break;
      case 4: out.balance_status_txt="Auto Balance"; break;
      default: out.balance_status_txt="Unknown"; break;
    }
  }

  // High cell voltage = [115,116,117]
  out.high_cell_num = hex_array[115];
  out.high_cell_voltage = ((hex_array[116] << 8) | hex_array[117]) / 1000.0f;

  // Low cell voltage = [118,119,120]
  out.low_cell_num = hex_array[118];
  out.low_cell_voltage = ((hex_array[119] << 8) | hex_array[120]) / 1000.0f;

  return out;
}
