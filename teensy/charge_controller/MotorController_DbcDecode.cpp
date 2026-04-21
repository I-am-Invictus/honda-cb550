#include "MotorController_DbcTypes.h"

namespace mcdbc {

static inline uint16_t u16_le(const uint8_t *d) {
  return static_cast<uint16_t>(d[0]) | (static_cast<uint16_t>(d[1]) << 8);
}

String errorSummary(const Msg1_0x0CF11E05 &m) {
  String s;
  auto add = [&](const char *txt) {
    if (s.length()) s += ", ";
    s += txt;
  };
  if (m.err0_identification_error) add("Identification");
  if (m.err1_over_voltage) add("OverVoltage");
  if (m.err2_low_voltage) add("LowVoltage");
  if (m.err4_stall) add("Stall");
  if (m.err5_internal_volts_fault) add("InternalVolts");
  if (m.err6_over_temperature) add("ControllerOverTemp");
  if (m.err7_throttle_error_at_powerup) add("ThrottleAtPowerUp");
  if (m.err9_internal_reset) add("InternalReset");
  if (m.err10_hall_throttle_open_or_short) add("HallThrottleFault");
  if (m.err11_angle_sensor_error) add("AngleSensor");
  if (m.err14_motor_over_temperature) add("MotorOverTemp");
  if (m.err15_hall_galvanometer_sensor_error) add("HallGalvanometer");
  if (!s.length()) s = "None";
  return s;
}

String feedbackStatusToString(uint8_t v) {
  switch (v & 0x03) {
    case 0: return "stationary";
    case 1: return "forward";
    case 2: return "backward";
    default: return "reserved";
  }
}

String commandStatusToString(uint8_t v) {
  switch (v & 0x03) {
    case 0: return "neutral";
    case 1: return "forward";
    case 2: return "backward";
    default: return "reserved";
  }
}

static void decodeMsg1(const uint8_t *buf, uint8_t len, Msg1_0x0CF11E05 &out) {
  if (len < 8) return;

  out.speed_rpm_raw = u16_le(&buf[0]);
  out.speed_rpm = static_cast<float>(out.speed_rpm_raw);

  out.motor_current_raw = u16_le(&buf[2]);
  out.motor_current_A = static_cast<float>(out.motor_current_raw) / 10.0f;

  out.battery_voltage_raw = u16_le(&buf[4]);
  out.battery_voltage_V = static_cast<float>(out.battery_voltage_raw) / 10.0f;

  out.error_code = u16_le(&buf[6]);

  out.err0_identification_error = out.error_code & (1u << 0);
  out.err1_over_voltage = out.error_code & (1u << 1);
  out.err2_low_voltage = out.error_code & (1u << 2);
  out.err3_reserved = out.error_code & (1u << 3);
  out.err4_stall = out.error_code & (1u << 4);
  out.err5_internal_volts_fault = out.error_code & (1u << 5);
  out.err6_over_temperature = out.error_code & (1u << 6);
  out.err7_throttle_error_at_powerup = out.error_code & (1u << 7);
  out.err8_reserved = out.error_code & (1u << 8);
  out.err9_internal_reset = out.error_code & (1u << 9);
  out.err10_hall_throttle_open_or_short = out.error_code & (1u << 10);
  out.err11_angle_sensor_error = out.error_code & (1u << 11);
  out.err12_reserved = out.error_code & (1u << 12);
  out.err13_reserved = out.error_code & (1u << 13);
  out.err14_motor_over_temperature = out.error_code & (1u << 14);
  out.err15_hall_galvanometer_sensor_error = out.error_code & (1u << 15);
}

static void decodeMsg2(const uint8_t *buf, uint8_t len, Msg2_0x0CF11F05 &out) {
  if (len < 8) return;

  out.throttle_raw = buf[0];
  out.throttle_V = (static_cast<float>(out.throttle_raw) * 5.0f) / 255.0f;

  out.controller_temp_raw = buf[1];
  out.controller_temp_C = static_cast<float>(out.controller_temp_raw) - 40.0f;

  out.motor_temp_raw = buf[2];
  out.motor_temp_C = static_cast<float>(out.motor_temp_raw) - 30.0f;

  out.status_of_controller = buf[4];
  out.status_of_switch_signals = buf[5];

  out.feedback_status = out.status_of_controller & 0x03;
  out.command_status = (out.status_of_controller >> 2) & 0x03;

  out.boost_switch = (out.status_of_switch_signals >> 7) & 0x01;
  out.foot_switch = (out.status_of_switch_signals >> 6) & 0x01;
  out.forward_switch = (out.status_of_switch_signals >> 5) & 0x01;
  out.backward_switch = (out.status_of_switch_signals >> 4) & 0x01;
  out.brake_12V_switch = (out.status_of_switch_signals >> 3) & 0x01;
  out.hall_c = (out.status_of_switch_signals >> 2) & 0x01;
  out.hall_b = (out.status_of_switch_signals >> 1) & 0x01;
  out.hall_a = (out.status_of_switch_signals >> 0) & 0x01;
}

bool decode(uint32_t can_id, const uint8_t *buf, uint8_t len, AnyMessage &out) {
  out.type = AnyMessage::Type::Unknown;
  if (!buf || len < 8) return false;

  switch (can_id) {
    case Msg1_0x0CF11E05::kCanId:
      decodeMsg1(buf, len, out.msg1);
      out.type = AnyMessage::Type::Msg1_0x0CF11E05;
      return true;

    case Msg2_0x0CF11F05::kCanId:
      decodeMsg2(buf, len, out.msg2);
      out.type = AnyMessage::Type::Msg2_0x0CF11F05;
      return true;

    default:
      return false;
  }
}

} // namespace mcdbc
