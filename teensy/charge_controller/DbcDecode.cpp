#include "DbcDecode.h"

namespace dbc {

// ----------------------------- low-level helpers ---------------------------
static inline uint16_t le_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t le_i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t le_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ------------------------------ value tables -------------------------------
String toString(BatteryStatus v) {
  return (v == BatteryStatus::Enabled) ? "Enabled" : "Disabled";
}
String toString(ChargerHardwareShutdownStatus v) {
  return (v == ChargerHardwareShutdownStatus::ShutDown) ? "Charger hardware has shut down" : "Charger is running normally";
}
String toString(ChargerDeratingStatus v) {
  return (v == ChargerDeratingStatus::Derating) ? "Charger is derating output" : "Charger is not derating";
}
String toString(ACConnectionStatus v) {
  return (v == ACConnectionStatus::ACDetected) ? "AC Detected" : "No AC Detected";
}
String toString(ChargerStatus v) {
  return (v == ChargerStatus::Enabled) ? "Enabled" : "Disabled";
}
String toString(OverrideStatus v) {
  return (v == OverrideStatus::Enabled) ? "Enabled" : "Disabled";
}
String toString(ChargeIndication v) {
  switch (v) {
    case ChargeIndication::Inactive: return "Inactive";
    case ChargeIndication::LessThan80: return "Less than 80%";
    case ChargeIndication::MoreThan80: return "More than 80%";
    case ChargeIndication::Finishing: return "Finishing";
    case ChargeIndication::Complete: return "Complete";
    case ChargeIndication::Resting: return "Resting";
    case ChargeIndication::Equalize: return "Equalize";
    case ChargeIndication::PowerSupplyMode: return "Power Supply Mode";
  }
  return "Unknown";
}
String toString(BattChargeCycleType v) {
  switch (v) {
    case BattChargeCycleType::NoActiveCycle: return "No Active Cycle";
    case BattChargeCycleType::Charge: return "Charge";
    case BattChargeCycleType::Description_0x2: return "Description for the value '0x2'";
  }
  return "Unknown";
}
String toString(NMTCommand v) {
  return (v == NMTCommand::Start) ? "Start" : "Unknown";
}
String toString(HeartbeatState v) {
  switch (v) {
    case HeartbeatState::Operational: return "Operational";
    case HeartbeatState::PreOperational: return "Pre-operational";
  }
  return "Unknown";
}

String currentErrorToText(uint32_t c) {
  // Subset from the DBC VAL tables (add more as needed)
  switch (c) {
    case 394301440u:  return "E-0-2-3 High AC voltage error ( >270VAC ) 9000h External error – generic";
    case 411045888u:  return "E-0-2-4 Charger failed to initialize 1000h Generic error";
    case 427855872u:  return "E-0-2-5 Low AC voltage oscillation error 9000h External error – generic";
    case 444596224u:  return "E-0-2-6 USB Script Error 0000h error";
    case 461373440u:  return "E-0-2-7 USB Over Current 0000h error";
    case 478154752u:  return "E-0-2-8 Incompatible algorithm error 1000h Generic error";
    case 494964736u:  return "E-0-2-9 Communication CAN-bus error 9000h External error – generic";
    case 511738160u:  return "E-0-3-0 Communication battery module error 8130h Monitoring – Comms – Heartbeat Error";
    case 528486400u:  return "E-0-3-1 Reference out of range error 1000h Generic error";
    case 545292592u:  return "E-0-3-2 Communication heartbeat lost error 8130h Monitoring – Comms – Heartbeat Error";
    case 562040832u:  return "E-0-3-3 Target voltage configuration too high 1000h Generic error";
    case 578818048u:  return "E-0-3-4 Battery capacity configuration not set 1000h Generic error";
    case 595595264u:  return "E-0-3-5 Target voltage configuration too low 1000h Generic error";
    case 612405248u:  return "E-0-3-6 Battery temperature sensor not installed 9000h External error – generic";
    case 629170176u:  return "E-0-3-7 CAN Download Failed 6000h SW Generic error";
    case 645959680u:  return "E-0-3-8 Fan error 9000h External error – generic";
    case 662704128u:  return "E-0-3-9 Button stuck down 1000h Generic error";
    case 679481344u:  return "E-0-4-0 Fan Supply Voltage Low 1000h Generic error";
    case 696279040u:  return "E-0-4-1 Software Internal Error 6000h SW Generic error";
    case 713056256u:  return "E-0-4-2 CAN Configuration Error 6000h SW Generic error";
    case 729845760u:  return "E-0-4-3 PDO CRC Error 9000h External error – generic";
    case 746622976u:  return "E-0-4-4 PDO Sequence Count Error 9000h External error – generic";
    case 763400192u:  return "E-0-4-5 Battery Disconnected Alarm 9000h External error - generic";
    case 780173840u:  return "E-0-4-6 Invalid PDO Length 8210h Monitoring – Protocol – PDO Length Error";
    case 29380608u:   return "F-0-0-1 Output Stage Error 5000h CANopen Device Hardware";
    case 46157824u:   return "F-0-0-2 Input Stage Error 5000h CANopen Device Hardware";
    case 62935040u:   return "F-0-0-3 Input Stage Error 5000h CANopen Device Hardware";
    case 79712256u:   return "F-0-0-4 Current Measurement Error 5000h CANopen Device Hardware";
    case 96489472u:   return "F-0-0-5 DC Output Relay Test Error (High voltage across closed relay) 5000h CANopen Device Hardware";
    case 1342179008u: return "F-0-0-6 Output Current Error 5000h CANopen Device Hardware";
    default:          return String("Unknown error code: ") + String(c);
  }
}

// ------------------------------- decoders ----------------------------------
static bool decode_RPDO2_30A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::RPDO2_30A;
  auto &m = out.rpdo2_30a;

  // positions (Intel): current @ bytes [2..3], voltage @ [4..5], temp @ [6..7]
  const uint16_t cur_raw = le_u16(&d[2]);      // 16|16
  const uint16_t volt_raw = le_u16(&d[4]);     // 32|16
  const int16_t  temp_raw = le_i16(&d[6]);     // 48|16 (signed)

  m.charging_current_A = cur_raw * 0.00390625f;  // /256
  m.battery_voltage_V  = volt_raw * 0.00390625f; // /256
  m.temperature_C      = (temp_raw * 0.125f) - 40.0f;

  return true;
}

static bool decode_RPDO1_20A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::RPDO1_20A;
  auto &m = out.rpdo1_20a;

  m.battery_soc_pct   = d[1];                           // 8|8
  m.charge_cycle_type = static_cast<BattChargeCycleType>(d[2]); // 16|8
  const uint16_t vreq_raw = le_u16(&d[3]);              // 24|16
  const uint16_t ireq_raw = le_u16(&d[5]);              // 40|16
  m.voltage_request_V = vreq_raw * 0.00390625f;         // /256
  m.current_request_A = ireq_raw * 0.0625f;             // /16
  m.battery_status    = static_cast<BatteryStatus>(d[7]); // 56|8

  return true;
}

static bool decode_TPDO3_38A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::TPDO3_38A;
  auto &m = out.tpdo3_38a;

  m.current_error_raw = le_u32(&d[0]);    // 0|32
  const uint16_t ac_raw = le_u16(&d[4]);  // 32|16
  m.ac_voltage_VAC = ac_raw * 0.0625f;    // /16
  m.charger_soc_pct = d[6];               // 48|8
  m.current_error_text = currentErrorToText(m.current_error_raw);

  return true;
}

static bool decode_TPDO2_28A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::TPDO2_28A;
  auto &m = out.tpdo2_28a;

  const uint16_t t_raw  = le_u16(&d[0]);  // 0|16
  const uint32_t ah_raw = le_u32(&d[2]);  // 16|32
  const uint16_t wh_raw = le_u16(&d[6]);  // 48|16

  m.elapsed_time_s = t_raw * 10.0f;
  m.ah_returned_Ah = ah_raw * 0.125f;     // /8
  m.wh_returned_Wh = wh_raw * 0.0625f;    // /16

  return true;
}

static bool decode_TPDO1_18A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::TPDO1_18A;
  auto &m = out.tpdo1_18a;

  const uint16_t cur_raw = le_u16(&d[0]); // 0|16
  const uint16_t v_raw   = le_u16(&d[2]); // 16|16
  m.charging_current_A = cur_raw * 0.00390625f;
  m.battery_voltage_V  = v_raw   * 0.00390625f;

  // Bits starting at bit 34 live in byte 4 (bit2..), byte5, etc. (Intel layout)
  // Byte index map:
  //  byte4 = d[4], byte5 = d[5]
  // bit positions (LSB0):
  //   bit34 -> d[4] bit2, bit35 -> d[4] bit3, bit36 -> d[4] bit4, bit37 -> d[4] bit5
  //   bits38..39 -> d[4] bits6..7 (Override 2-bit)
  //   bits40..43 -> d[5] bits0..3 (Charge_Indication 4-bit)
  //   bits44..47 -> d[5] bits4..7 (Charge_Cycle_Type 4-bit)
  const uint8_t b4 = d[4];
  const uint8_t b5 = d[5];

  m.hw_shutdown      = ((b4 >> 2) & 0x01) ? ChargerHardwareShutdownStatus::ShutDown : ChargerHardwareShutdownStatus::Running;
  m.derating         = ((b4 >> 3) & 0x01) ? ChargerDeratingStatus::Derating : ChargerDeratingStatus::NotDerating;
  m.ac_status        = ((b4 >> 4) & 0x01) ? ACConnectionStatus::ACDetected : ACConnectionStatus::NoAC;
  m.charger_status   = ((b4 >> 5) & 0x01) ? ChargerStatus::Enabled : ChargerStatus::Disabled;

  const uint8_t override2 = (b4 >> 6) & 0x03;
  m.override_status = (override2 ? OverrideStatus::Enabled : OverrideStatus::Disabled);

  const uint8_t ci =  (b5 & 0x0F);       // bits 0..3
  const uint8_t cct = (b5 >> 4) & 0x0F;  // bits 4..7
  m.charge_indication = static_cast<ChargeIndication>(ci);
  // The table only defines 0..2 explicitly; map higher safely into enum domain
  m.charge_cycle_type = static_cast<BattChargeCycleType>(cct <= 2 ? cct : 0);

  return true;
}

static bool decode_NMT_000(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 2) return false;
  out.type = AnyMessage::Type::NMT_Start;
  out.nmt_start.command = static_cast<NMTCommand>(d[0]);
  out.nmt_start.node_id = d[1];
  return true;
}

static bool decode_FaultReg_08A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 8) return false;
  out.type = AnyMessage::Type::FaultReg_08A;
  for (int i=0;i<8;++i) out.fault_reg.raw[i] = d[i];
  return true;
}

static bool decode_HB_701(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 1) return false;
  out.type = AnyMessage::Type::HB_701;
  out.hb_701.state = static_cast<HeartbeatState>(d[0]);
  return true;
}

static bool decode_HB_70A(const uint8_t d[8], uint8_t dlc, AnyMessage &out) {
  if (dlc < 1) return false;
  out.type = AnyMessage::Type::HB_70A;
  out.hb_70a.state = static_cast<HeartbeatState>(d[0]);
  return true;
}

// --------------------------------- API -------------------------------------
bool decode(uint32_t can_id, const uint8_t data[8], uint8_t dlc, AnyMessage &out) {
  switch (can_id) {
    case ID_RPDO2_0x30A:          return decode_RPDO2_30A(data, dlc, out);
    case ID_RPDO1_0x20A:          return decode_RPDO1_20A(data, dlc, out);
    case ID_TPDO3_0x38A:          return decode_TPDO3_38A(data, dlc, out);
    case ID_TPDO2_0x28A:          return decode_TPDO2_28A(data, dlc, out);
    case ID_TPDO1_0x18A:          return decode_TPDO1_18A(data, dlc, out);
    case ID_NMT_Start:            return decode_NMT_000(data, dlc, out);
    case ID_Fault_Register_0x08A: return decode_FaultReg_08A(data, dlc, out);
    case ID_Heartbeat_Response:   return decode_HB_701(data, dlc, out);
    case ID_Heartbeat_0x70A:      return decode_HB_70A(data, dlc, out);
    default:
      out.type = AnyMessage::Type::None;
      return false;
  }
}

} // namespace dbc
