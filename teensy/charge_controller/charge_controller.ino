#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include "DbcTypes.h"
#include "DbcDecode.h"
#include "BmsDecoder.h"
#include "MotorController_DbcTypes.h"
#include "MotorController_DbcDecode.h"


// Use Serial1 for TTL RX/TX (pins 0=RX1, 1=TX1 on Teensy 4.1)
#define BMS_SERIAL Serial1

#define TELEMETRY_SERIAL Serial2

// BMS request bytes
const uint8_t BMS_REQUEST[6] = {0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00};

// Teensy 4.1 CAN1: CRX1=22, CTX1=23
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;   // second CAN bus for motor controller


// -------------------- Central SystemState --------------------
struct SystemState {
  struct { bool valid=false; dbc::DeltaQ_RPDO2_0x30A data; } rpdo2_30a;
  struct { bool valid=false; dbc::DeltaQ_RPDO1_0x20A data; } rpdo1_20a;
  struct { bool valid=false; dbc::DeltaQ_TPDO3_0x38A data; } tpdo3_38a;
  struct { bool valid=false; dbc::DeltaQ_TPDO2_0x28A data; } tpdo2_28a;
  struct { bool valid=false; dbc::DeltaQ_TPDO1_0x18A data; } tpdo1_18a;
  struct { bool valid=false; dbc::NMT_Start_0x000 data; } nmt;
  struct { bool valid=false; dbc::Fault_Register_0x08A data; } faultreg;
  struct { bool valid=false; dbc::Heartbeat_Response_0x701 data; } hb701;
  struct { bool valid=false; dbc::Heartbeat_0x70A data; } hb70a;

  struct { bool valid=false; BmsData data; uint32_t last_update_ms=0; } bms;
};
SystemState sysState;

// -------------------- Motor State --------------------------
struct MotorState {
  struct { bool valid=false; mcdbc::Msg1_0x0CF11E05 data; } msg1;
  struct { bool valid=false; mcdbc::Msg2_0x0CF11F05 data; } msg2;
  uint32_t last_update_ms = 0;
};

MotorState motorState;
elapsedMillis telemetryTimer;
constexpr uint32_t TELEMETRY_PERIOD_MS = 500;

// -------------------- IDs and constants --------------------
constexpr uint8_t BATTERY_NODE_ID  = 0x01;
constexpr uint8_t CHARGER_NODE_ID  = 0x0A;

constexpr uint32_t NMT_ID          = 0x000;
constexpr uint32_t HEARTBEAT_BASE  = 0x700;
constexpr uint32_t CHARGER_HB_ID   = HEARTBEAT_BASE + CHARGER_NODE_ID; // 0x70A
constexpr uint32_t BATTERY_HB_ID   = HEARTBEAT_BASE + BATTERY_NODE_ID; // 0x701
constexpr uint32_t RPDO1_ID        = 0x200 + CHARGER_NODE_ID;          // 0x20A

// Match actual charger config. Delta-Q example/default is 125 kbps.
constexpr uint32_t CAN_BAUD = 500000;

constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;
constexpr uint32_t RPDO1_PERIOD_MS     = 250;
constexpr uint32_t BMS_REQUEST_PERIOD_MS = 1000;
constexpr uint32_t STATUS_PRINT_MS     = 2000;

// Open-loop charging request
constexpr float TARGET_VOLTAGE_V       = 82.0f;  // 20s * 4.10 V/cell
constexpr float TARGET_CURRENT_A       = 10.0f;  // conservative default
constexpr float MAX_ALLOWED_VOLTAGE_V  = 82.0f;
constexpr float MAX_ALLOWED_CURRENT_A  = 15.0f;

// Optional BMS safety cutoffs
constexpr float MAX_CELL_VOLTAGE_V     = 4.10f;
constexpr float MAX_PACK_VOLTAGE_V     = 82.0f;

// BMS frame handling
constexpr size_t BMS_FRAME_LEN = 121;
uint8_t bmsRxBuf[160];
size_t bmsRxLen = 0;

// -------------------- Startup / run state --------------------
enum class ChargerControlState : uint8_t {
  WAIT_FOR_CHARGER_HEARTBEAT = 0,
  SEND_INITIAL_HEARTBEAT,
  SEND_NMT_START,
  SEND_RPDO1_NOT_READY,
  RUN_CHARGING,
  STOPPING,
  FAULTED
};

ChargerControlState controlState = ChargerControlState::WAIT_FOR_CHARGER_HEARTBEAT;

bool chargerHeartbeatSeen = false;
uint32_t lastChargerHeartbeatMs = 0;
elapsedMillis hbTimer;
elapsedMillis rpdoTimer;
elapsedMillis stateTimer;
elapsedMillis bmsRequestTimer;

// -------------------- Helpers --------------------
static uint16_t encodeVoltage256(float volts) {
  if (volts < 0.0f) volts = 0.0f;
  return (uint16_t)lroundf(volts * 256.0f);
}

static uint16_t encodeCurrent16(float amps) {
  if (amps < 0.0f) amps = 0.0f;
  return (uint16_t)lroundf(amps * 16.0f);
}

static bool chargerFaultActive() {
  if (!sysState.tpdo1_18a.valid) return false;
  const auto &d = sysState.tpdo1_18a.data;
  return d.hw_shutdown != 0;
}

static bool bmsShouldStopCharge() {
  if (!sysState.bms.valid) return false;

  const auto &b = sysState.bms.data;

  if (b.pack_voltage_V >= MAX_PACK_VOLTAGE_V) {
    Serial.printf("BMS stop: pack voltage high: %.3f V\n", b.pack_voltage_V);
    return true;
  }

  if (b.high_cell_voltage >= MAX_CELL_VOLTAGE_V) {
    Serial.printf("BMS stop: high cell %u at %.4f V\n",
                  b.high_cell_num, b.high_cell_voltage);
    return true;
  }

  return false;
}

// -------------------- CAN RX Callback --------------------
void onRx(const CAN_message_t &msg) {
  if (msg.id == CHARGER_HB_ID && msg.len >= 1) {
    chargerHeartbeatSeen = true;
    lastChargerHeartbeatMs = millis();
  }

  dbc::AnyMessage decoded;
  if (dbc::decode(msg.id, msg.buf, msg.len, decoded)) {
    switch (decoded.type) {
      case dbc::AnyMessage::Type::RPDO2_30A:
        sysState.rpdo2_30a.data = decoded.rpdo2_30a;
        sysState.rpdo2_30a.valid = true;
        break;
      case dbc::AnyMessage::Type::RPDO1_20A:
        sysState.rpdo1_20a.data = decoded.rpdo1_20a;
        sysState.rpdo1_20a.valid = true;
        break;
      case dbc::AnyMessage::Type::TPDO3_38A:
        sysState.tpdo3_38a.data = decoded.tpdo3_38a;
        sysState.tpdo3_38a.valid = true;
        break;
      case dbc::AnyMessage::Type::TPDO2_28A:
        sysState.tpdo2_28a.data = decoded.tpdo2_28a;
        sysState.tpdo2_28a.valid = true;
        break;
      case dbc::AnyMessage::Type::TPDO1_18A:
        sysState.tpdo1_18a.data = decoded.tpdo1_18a;
        sysState.tpdo1_18a.valid = true;
        break;
      case dbc::AnyMessage::Type::NMT_Start:
        sysState.nmt.data = decoded.nmt_start;
        sysState.nmt.valid = true;
        break;
      case dbc::AnyMessage::Type::FaultReg_08A:
        sysState.faultreg.data = decoded.fault_reg;
        sysState.faultreg.valid = true;
        break;
      case dbc::AnyMessage::Type::HB_701:
        sysState.hb701.data = decoded.hb_701;
        sysState.hb701.valid = true;
        break;
      case dbc::AnyMessage::Type::HB_70A:
        sysState.hb70a.data = decoded.hb_70a;
        sysState.hb70a.valid = true;
        break;
      default:
        break;
    }
  }
}

void onMotorCanRx(const CAN_message_t &msg) {
  if (!msg.flags.extended) return;

  mcdbc::AnyMessage decoded;
  if (mcdbc::decode(msg.id, msg.buf, msg.len, decoded)) {
    switch (decoded.type) {
      case mcdbc::AnyMessage::Type::Msg1_0x0CF11E05:
        motorState.msg1.data = decoded.msg1;
        motorState.msg1.valid = true;
        motorState.last_update_ms = millis();
        break;

      case mcdbc::AnyMessage::Type::Msg2_0x0CF11F05:
        motorState.msg2.data = decoded.msg2;
        motorState.msg2.valid = true;
        motorState.last_update_ms = millis();
        break;

      default:
        break;
    }
  }
}

// -------------------- TX helpers --------------------
void sendNMTStart() {
  CAN_message_t msg;
  msg.id = NMT_ID;
  msg.len = 2;
  msg.buf[0] = 0x01;
  msg.buf[1] = CHARGER_NODE_ID;
  can1.write(msg);
  Serial.println(">> Sent NMT Start to charger");
}

void sendHeartbeat() {
  CAN_message_t msg;
  msg.id = BATTERY_HB_ID;
  msg.len = 1;
  msg.buf[0] = 0x05;
  can1.write(msg);
}

void sendRPDO1(bool batteryReady, float voltageV, float currentA, uint8_t socPct = 0, uint8_t externalOverride0 = 0) {
  float v = voltageV;
  float i = currentA;

  if (v > MAX_ALLOWED_VOLTAGE_V) v = MAX_ALLOWED_VOLTAGE_V;
  if (i > MAX_ALLOWED_CURRENT_A) i = MAX_ALLOWED_CURRENT_A;
  if (v < 0.0f) v = 0.0f;
  if (i < 0.0f) i = 0.0f;

  const uint16_t vreq_raw = encodeVoltage256(v);
  const uint16_t ireq_raw = encodeCurrent16(i);

  CAN_message_t msg;
  msg.id = RPDO1_ID;
  msg.len = 8;
  memset(msg.buf, 0, sizeof(msg.buf));

  msg.buf[0] = 0x00;
  msg.buf[1] = socPct;
  msg.buf[2] = externalOverride0;
  msg.buf[3] = (uint8_t)(vreq_raw & 0xFF);
  msg.buf[4] = (uint8_t)((vreq_raw >> 8) & 0xFF);
  msg.buf[5] = (uint8_t)(ireq_raw & 0xFF);
  msg.buf[6] = (uint8_t)((ireq_raw >> 8) & 0xFF);
  msg.buf[7] = batteryReady ? 0x01 : 0x00;

  can1.write(msg);
}

void sendSafeStop() {
  sendRPDO1(true, TARGET_VOLTAGE_V, 0.0f, 0, 0);
  delay(20);
  sendRPDO1(false, TARGET_VOLTAGE_V, 0.0f, 0, 0);
  Serial.println(">> Sent safe stop sequence");
}

// -------------------- BMS helpers --------------------
void requestBmsFrame() {
  BMS_SERIAL.write(BMS_REQUEST, sizeof(BMS_REQUEST));
  BMS_SERIAL.flush();
}

void readBmsSerial() {
  while (BMS_SERIAL.available() > 0) {
    int c = BMS_SERIAL.read();
    if (c < 0) {
      break;
    }

    if (bmsRxLen < sizeof(bmsRxBuf)) {
      bmsRxBuf[bmsRxLen++] = (uint8_t)c;
    } else {
      // Overflow protection: drop buffer and start over
      bmsRxLen = 0;
      Serial.println("BMS RX overflow, buffer reset");
      break;
    }
  }

  // If at least one full frame is available, decode the most recent 121 bytes.
  while (bmsRxLen >= BMS_FRAME_LEN) {
    BmsData decoded = decodeBmsMessage(bmsRxBuf, BMS_FRAME_LEN);
    sysState.bms.data = decoded;
    sysState.bms.valid = true;
    sysState.bms.last_update_ms = millis();

    // Remove the consumed frame
    size_t remaining = bmsRxLen - BMS_FRAME_LEN;
    if (remaining > 0) {
      memmove(bmsRxBuf, bmsRxBuf + BMS_FRAME_LEN, remaining);
    }
    bmsRxLen = remaining;

    Serial.printf("[BMS] Pack=%.2f V, Current=%.2f A, SOC=%u%%, HighCell=%u:%.3f V, LowCell=%u:%.3f V\n",
                  decoded.pack_voltage_V,
                  decoded.pack_current_A,
                  decoded.soc_pct,
                  decoded.high_cell_num,
                  decoded.high_cell_voltage,
                  decoded.low_cell_num,
                  decoded.low_cell_voltage);
  }
}

// Send Telemetry to the Screen
float rpmToMph(float rpm) {
  // Replace with your actual drivetrain constants
  constexpr float WHEEL_CIRCUMFERENCE_M = 1.884f; // example ~24 inch tire
  constexpr float GEAR_RATIO = 10.0f;             // motor rev / wheel rev

  float wheel_rpm = rpm / GEAR_RATIO;
  float meters_per_min = wheel_rpm * WHEEL_CIRCUMFERENCE_M;
  float mph = meters_per_min * 0.0372823f;
  return mph;
}

void sendTelemetryLine() {
  if (!motorState.msg1.valid) return;

  float rpm = motorState.msg1.data.speed_rpm;
  float voltage = motorState.msg1.data.battery_voltage_V;
  float current = motorState.msg1.data.motor_current_A;

  // If you want battery current instead of motor current, use BMS current instead.
  float power = voltage * current;

  float soc = sysState.bms.valid ? sysState.bms.data.soc_pct : 0.0f;
  float btemp = sysState.bms.valid ? sysState.bms.data.mos_temperature_C : 0.0f;
  float mtemp = motorState.msg2.valid ? motorState.msg2.data.motor_temp_C : 0.0f;
  float mph = rpmToMph(rpm);

  TELEMETRY_SERIAL.printf("%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f\r\n",
                          mph, voltage, current, power, soc, rpm, btemp, mtemp);
}

// -------------------- Arduino Setup/Loop --------------------
void setup() {
  Serial.begin(115200);
  BMS_SERIAL.begin(115200);
  TELEMETRY_SERIAL.begin(115200);

  while (!Serial && millis() < 3000) {}
  Serial.println("Teensy 4.1 Delta-Q open-loop battery simulator with BMS polling");

  if (TARGET_VOLTAGE_V > MAX_ALLOWED_VOLTAGE_V || TARGET_CURRENT_A > MAX_ALLOWED_CURRENT_A) {
    Serial.println("ERROR: Target voltage/current exceeds configured safety limits.");
    controlState = ChargerControlState::FAULTED;
  }

  can1.begin();
  can1.setBaudRate(CAN_BAUD);
  can1.onReceive(onRx);
  can1.enableMBInterrupts();

  can2.begin();
  can2.setBaudRate(250000);   // Kelly protocol PDF says 250 kbps
  can2.onReceive(onMotorCanRx);
  can2.enableMBInterrupts();

  hbTimer = 0;
  rpdoTimer = 0;
  stateTimer = 0;
  bmsRequestTimer = 0;

  Serial.printf("CAN baud: %lu\n", CAN_BAUD);
  Serial.println("Waiting for charger heartbeat 0x70A...");
}

void loop() {
  can1.events();
  can2.events();
  readBmsSerial();

  if (bmsRequestTimer >= BMS_REQUEST_PERIOD_MS) {
    bmsRequestTimer = 0;
    requestBmsFrame();
  }

  if (controlState != ChargerControlState::WAIT_FOR_CHARGER_HEARTBEAT &&
      controlState != ChargerControlState::FAULTED &&
      controlState != ChargerControlState::STOPPING &&
      hbTimer >= HEARTBEAT_PERIOD_MS) {
    hbTimer = 0;
    sendHeartbeat();
  }

  if (controlState == ChargerControlState::RUN_CHARGING) {
    if (chargerFaultActive()) {
      Serial.println("FAULT: Charger reported shutdown/fault condition.");
      controlState = ChargerControlState::STOPPING;
    }

    if (chargerHeartbeatSeen && (millis() - lastChargerHeartbeatMs > 3000)) {
      Serial.println("FAULT: Lost charger heartbeat.");
      controlState = ChargerControlState::STOPPING;
    }

    if (bmsShouldStopCharge()) {
      Serial.println("BMS requested stop.");
      controlState = ChargerControlState::STOPPING;
    }
  }

  switch (controlState) {
    case ChargerControlState::WAIT_FOR_CHARGER_HEARTBEAT:
      if (chargerHeartbeatSeen) {
        Serial.println("<< Saw charger heartbeat 0x70A");
        sendHeartbeat();
        hbTimer = 0;
        stateTimer = 0;
        controlState = ChargerControlState::SEND_NMT_START;
      }
      break;

    case ChargerControlState::SEND_NMT_START:
      if (stateTimer >= 50) {
        sendNMTStart();
        stateTimer = 0;
        controlState = ChargerControlState::SEND_RPDO1_NOT_READY;
      }
      break;

    case ChargerControlState::SEND_RPDO1_NOT_READY:
      if (stateTimer >= 50) {
        sendRPDO1(false, TARGET_VOLTAGE_V, TARGET_CURRENT_A, 0, 0);
        rpdoTimer = 0;
        stateTimer = 0;
        controlState = ChargerControlState::RUN_CHARGING;
      }
      break;

    case ChargerControlState::RUN_CHARGING: {
      if (rpdoTimer >= RPDO1_PERIOD_MS) {
        rpdoTimer = 0;

        uint8_t socToSend = 0;
        if (sysState.bms.valid) {
          socToSend = sysState.bms.data.soc_pct;
        }

        sendRPDO1(true, TARGET_VOLTAGE_V, TARGET_CURRENT_A, socToSend, 0);
      }
      break;
    }

    case ChargerControlState::STOPPING:
      sendSafeStop();
      controlState = ChargerControlState::FAULTED;
      break;

    case ChargerControlState::FAULTED:
      break;

    default:
      break;
  }

  if (telemetryTimer >= TELEMETRY_PERIOD_MS) {
    telemetryTimer = 0;
    sendTelemetryLine();
  }

  static uint32_t t_last = 0;
  if (millis() - t_last > STATUS_PRINT_MS) {
    t_last = millis();

    Serial.printf("[STATE] %u, chargerHB=%s, lastHB=%lu ms ago\n",
                  (unsigned)controlState,
                  chargerHeartbeatSeen ? "yes" : "no",
                  chargerHeartbeatSeen ? (unsigned long)(millis() - lastChargerHeartbeatMs) : 0UL);

    if (sysState.tpdo1_18a.valid) {
      auto &d = sysState.tpdo1_18a.data;
      Serial.printf("[0x18A] I=%.2f A, V=%.2f V, HW=%s, Derating=%s, AC=%s, Charger=%s, Override=%s, Indication=%s, Cycle=%s\n",
                    d.charging_current_A,
                    d.battery_voltage_V,
                    dbc::toString(d.hw_shutdown).c_str(),
                    dbc::toString(d.derating).c_str(),
                    dbc::toString(d.ac_status).c_str(),
                    dbc::toString(d.charger_status).c_str(),
                    dbc::toString(d.override_status).c_str(),
                    dbc::toString(d.charge_indication).c_str(),
                    dbc::toString(d.charge_cycle_type).c_str());
    }

    if (sysState.bms.valid) {
      const auto &b = sysState.bms.data;
      Serial.printf("[BMS] Pack=%.2fV Current=%.2fA SOC=%u%% MOS=%.1fC Bal=%.1fC ChgMOS=%s DsgMOS=%s Bal=%s HighCell=%u %.3fV LowCell=%u %.3fV\n",
                    b.pack_voltage_V,
                    b.pack_current_A,
                    b.soc_pct,
                    b.mos_temperature_C,
                    b.balance_temperature_C,
                    b.charge_mos_status_txt.c_str(),
                    b.discharge_mos_status_txt.c_str(),
                    b.balance_status_txt.c_str(),
                    b.high_cell_num,
                    b.high_cell_voltage,
                    b.low_cell_num,
                    b.low_cell_voltage);
    }
    
    if (motorState.msg1.valid) {
      Serial.printf("[MOTOR1] rpm=%.0f battV=%.1f motorA=%.1f err=0x%04X %s\n",
                    motorState.msg1.data.speed_rpm,
                    motorState.msg1.data.battery_voltage_V,
                    motorState.msg1.data.motor_current_A,
                    motorState.msg1.data.error_code,
                    mcdbc::errorSummary(motorState.msg1.data).c_str());
    }

    if (motorState.msg2.valid) {
      Serial.printf("[MOTOR2] throttle=%.2fV ctrlT=%.1fC motorT=%.1fC feedback=%s cmd=%s\n",
                    motorState.msg2.data.throttle_V,
                    motorState.msg2.data.controller_temp_C,
                    motorState.msg2.data.motor_temp_C,
                    mcdbc::feedbackStatusToString(motorState.msg2.data.feedback_status).c_str(),
                    mcdbc::commandStatusToString(motorState.msg2.data.command_status).c_str());
    }
  }
}