#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "DbcTypes.h"
#include "DbcDecode.h"
#include "BmsDecoder.h"

// Use Serial1 for TTL RX/TX (pins 0=RX1, 1=TX1 on Teensy 4.1)
#define BMS_SERIAL Serial1

// BMS request bytes
const uint8_t BMS_REQUEST[6] = {0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00};


// Teensy 4.1 CAN1: CRX1=22, CTX1=23
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

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
};
SystemState sysState;

// -------------------- IDs and constants --------------------
constexpr uint8_t BATTERY_NODE_ID  = 0x01;
constexpr uint8_t CHARGER_NODE_ID  = 0x0A;

constexpr uint32_t NMT_ID          = 0x000;
constexpr uint32_t HEARTBEAT_BASE  = 0x700;
constexpr uint32_t RPDO1_ID        = 0x20A;  // Battery → Charger

constexpr float HEARTBEAT_PERIOD_S = 0.5f;
constexpr float RPDO1_PERIOD_S     = 0.5f;   // 5 Hz

// -------------------- CAN RX Callback --------------------
void onRx(const CAN_message_t &msg) {
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
        sysState.hb70a.data = decoded.hb_70a;   // <-- fixed
        sysState.hb70a.valid = true;
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
  msg.buf[0] = 0x01;              // Start
  msg.buf[1] = CHARGER_NODE_ID;   // Target node
  can1.write(msg);
  Serial.println(">> Sent NMT Start to charger");
}

void sendHeartbeat() {
  CAN_message_t msg;
  msg.id = HEARTBEAT_BASE + BATTERY_NODE_ID;
  msg.len = 1;
  msg.buf[0] = 0x05;  // Operational
  can1.write(msg);
}

void sendRPDO1(bool firstZero) {
  // Build payload matching DBC for RPDO1 (0x20A)
  // Layout: b0 reserved, b1 SOC, b2 cycleType,
  //         b3-4 VoltageReq raw, b5-6 CurrentReq raw, b7 BatteryStatus
  CAN_message_t msg;
  msg.id = RPDO1_ID;
  msg.len = 8;
  memset(msg.buf, 0, 8);

  uint8_t soc_pct   = 50;    // example 50%
  uint8_t cycleType = 0;     // "Charge"
  float vreq = 82.0f;        // volts
  float ireq = 5.0f;         // amps

  uint16_t vreq_raw = (uint16_t)(vreq / 0.00390625f); // V*256
  uint16_t ireq_raw = (uint16_t)(ireq / 0.0625f);     // A*16

  msg.buf[0] = 0x00;         // reserved
  msg.buf[1] = soc_pct;
  msg.buf[2] = cycleType;
  msg.buf[3] = vreq_raw & 0xFF;
  msg.buf[4] = (vreq_raw >> 8) & 0xFF;
  msg.buf[5] = ireq_raw & 0xFF;
  msg.buf[6] = (ireq_raw >> 8) & 0xFF;
  msg.buf[7] = firstZero ? 0 : 1;  // BatteryStatus: 0 first, then 1

  can1.write(msg);
}

// -------------------- Timing state --------------------
elapsedMillis hbTimer;
elapsedMillis rpdoTimer;
bool firstRPDO1 = true;

// -------------------- Arduino Setup/Loop --------------------
void setup() {
  Serial.begin(115200);

  BMS_SERIAL.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("Teensy 4.1 CAN1 Battery Simulator");

  can1.begin();
  can1.setBaudRate(500000);
  can1.onReceive(onRx);
  can1.enableMBInterrupts();

  delay(5000);         // let bus settle
  // TODO add a step to wait until 0x701 heartbeat is seen on the bus before beginning
  sendNMTStart();     // Step 1: send NMT start
  hbTimer = 0;
  rpdoTimer = 0;
}

void loop() {
  can1.events();

  // Heartbeat at ~500ms
  if (hbTimer > (uint32_t)(HEARTBEAT_PERIOD_S * 1000)) {
    hbTimer = 0;
    sendHeartbeat();
  }

  // RPDO1 at ~500ms
  if (rpdoTimer > (uint32_t)(RPDO1_PERIOD_S * 1000)) {
    rpdoTimer = 0;
    sendRPDO1(firstRPDO1);
    if (firstRPDO1) firstRPDO1 = false;
  }

  // Example: print state once per second
  static uint32_t t_last = 0;
  if (millis() - t_last > 2000) {
    t_last = millis();
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
  }
}
