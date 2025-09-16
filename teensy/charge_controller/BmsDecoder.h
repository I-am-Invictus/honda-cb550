#pragma once
#include <Arduino.h>
#include <vector>

// Struct holding decoded BMS values
struct BmsData {
  float pack_voltage_V = 0;
  float pack_current_A = 0;
  uint8_t soc_pct = 0;

  std::vector<float> cell_voltages;  // each cell in V
  float mos_temperature_C = 0;
  float balance_temperature_C = 0;
  std::vector<float> external_temperatures; // 4 temps

  float physical_capacity_Ah = 0;
  float remaining_capacity_Ah = 0;
  float cyclic_capacity_Ah = 0;

  String discharge_mos_status_txt;
  uint8_t discharge_mos_status_code = 0;

  String charge_mos_status_txt;
  uint8_t charge_mos_status_code = 0;

  String balance_status_txt;
  uint8_t balance_status_code = 0;

  uint8_t high_cell_num = 0;
  float high_cell_voltage = 0;

  uint8_t low_cell_num = 0;
  float low_cell_voltage = 0;
};

// Main decode function
BmsData decodeBmsMessage(const uint8_t *bytes, size_t len);

