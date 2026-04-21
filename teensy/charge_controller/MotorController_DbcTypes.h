#pragma once
#include <Arduino.h>

namespace mcdbc {

struct Msg1_0x0CF11E05 {
  static constexpr uint32_t kCanId = 0x0CF11E05UL;
  static constexpr bool kExtended = true;

  uint16_t speed_rpm_raw = 0;
  float speed_rpm = 0.0f;

  uint16_t motor_current_raw = 0;
  float motor_current_A = 0.0f;

  uint16_t battery_voltage_raw = 0;
  float battery_voltage_V = 0.0f;

  uint16_t error_code = 0;

  bool err0_identification_error = false;
  bool err1_over_voltage = false;
  bool err2_low_voltage = false;
  bool err3_reserved = false;
  bool err4_stall = false;
  bool err5_internal_volts_fault = false;
  bool err6_over_temperature = false;
  bool err7_throttle_error_at_powerup = false;
  bool err8_reserved = false;
  bool err9_internal_reset = false;
  bool err10_hall_throttle_open_or_short = false;
  bool err11_angle_sensor_error = false;
  bool err12_reserved = false;
  bool err13_reserved = false;
  bool err14_motor_over_temperature = false;
  bool err15_hall_galvanometer_sensor_error = false;
};

struct Msg2_0x0CF11F05 {
  static constexpr uint32_t kCanId = 0x0CF11F05UL;
  static constexpr bool kExtended = true;

  uint8_t throttle_raw = 0;
  float throttle_V = 0.0f;   // 0-255 maps to 0-5V

  uint8_t controller_temp_raw = 0;
  float controller_temp_C = 0.0f; // raw - 40

  uint8_t motor_temp_raw = 0;
  float motor_temp_C = 0.0f;      // raw - 30

  uint8_t status_of_controller = 0;
  uint8_t status_of_switch_signals = 0;

  uint8_t feedback_status = 0; // bit1..0
  uint8_t command_status = 0;  // bit3..2

  bool boost_switch = false;
  bool foot_switch = false;
  bool forward_switch = false;
  bool backward_switch = false;
  bool brake_12V_switch = false;
  bool hall_c = false;
  bool hall_b = false;
  bool hall_a = false;
};

struct AnyMessage {
  enum class Type : uint8_t {
    Unknown = 0,
    Msg1_0x0CF11E05,
    Msg2_0x0CF11F05,
  } type = Type::Unknown;

  Msg1_0x0CF11E05 msg1;
  Msg2_0x0CF11F05 msg2;
};

String errorSummary(const Msg1_0x0CF11E05 &m);
String feedbackStatusToString(uint8_t v);
String commandStatusToString(uint8_t v);

} // namespace mcdbc
