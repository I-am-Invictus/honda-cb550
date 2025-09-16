#pragma once
#include <stdint.h>
#include <Arduino.h>  // for String (okay on Teensy/Arduino)

namespace dbc {

// ------------------------- Value Tables (from DBC) -------------------------
enum class BatteryStatus : uint8_t { Disabled = 0, Enabled = 1 };
enum class ChargerHardwareShutdownStatus : uint8_t { Running = 0, ShutDown = 1 };
enum class ChargerDeratingStatus : uint8_t { NotDerating = 0, Derating = 1 };
enum class ACConnectionStatus : uint8_t { NoAC = 0, ACDetected = 1 };
enum class ChargerStatus : uint8_t { Disabled = 0, Enabled = 1 };
enum class OverrideStatus : uint8_t { Disabled = 0, Enabled = 1 };

enum class ChargeIndication : uint8_t {
  Inactive = 0, LessThan80 = 1, MoreThan80 = 2, Finishing = 3,
  Complete = 4, Resting = 5, Equalize = 6, PowerSupplyMode = 7
};

enum class BattChargeCycleType : uint8_t {
  NoActiveCycle = 0, Charge = 1, Description_0x2 = 2
};

enum class NMTCommand : uint8_t { Start = 1 };

enum class HeartbeatState : uint8_t { Operational = 5, PreOperational = 127 };

// Fault / error is a u32 with specific codes -> string table (rendered by helper)

// ----------------------------- Message IDs --------------------------------
constexpr uint32_t ID_RPDO2_0x30A           = 0x30A; // Battery -> Charger
constexpr uint32_t ID_RPDO1_0x20A           = 0x20A; // Battery -> Charger
constexpr uint32_t ID_TPDO3_0x38A           = 0x38A; // Charger -> Battery
constexpr uint32_t ID_TPDO2_0x28A           = 0x28A; // Charger -> Battery
constexpr uint32_t ID_TPDO1_0x18A           = 0x18A; // Charger -> Battery
constexpr uint32_t ID_NMT_Start             = 0x000; // Battery -> Charger (2 bytes)
constexpr uint32_t ID_Fault_Register_0x08A  = 0x08A; // Charger -> Battery (8 bytes)
constexpr uint32_t ID_Heartbeat_Response    = 0x701; // Battery -> Charger (1 byte)
constexpr uint32_t ID_Heartbeat_0x70A       = 0x70A; // Charger -> Battery (1 byte)

// ---------------------------- Decoded structs ------------------------------
struct DeltaQ_RPDO2_0x30A {
  // SG_ Batt_Charging_Current : 16|16@1+ (0.00390625,0) "A"
  // SG_ Batt_Battery_Voltage : 32|16@1+ (0.00390625,0) "V"
  // SG_ Batt_Temperature     : 48|16@1- (0.125,-40)    "C"
  float charging_current_A {0.f};
  float battery_voltage_V  {0.f};
  float temperature_C      {0.f};
};

struct DeltaQ_RPDO1_0x20A {
  // SG_ Battery_SOC            : 8|8@1+  (1,0) "%"
  // SG_ Batt_Charge_Cycle_Type : 16|8@1+ (1,0)
  // SG_ Voltage_Request        : 24|16@1+ (0.00390625,0) "V"
  // SG_ Charge_Current_Request : 40|16@1+ (0.0625,0) "A"
  // SG_ Battery_Status         : 56|8@1+ (1,0)
  uint8_t                battery_soc_pct {0};
  BattChargeCycleType    charge_cycle_type {BattChargeCycleType::NoActiveCycle};
  float                  voltage_request_V {0.f};
  float                  current_request_A {0.f};
  BatteryStatus          battery_status {BatteryStatus::Disabled};
};

struct DeltaQ_TPDO3_0x38A {
  // SG_ Current_Error : 0|32@1+ (1,0)
  // SG_ AC_Voltage    : 32|16@1+ (0.0625,0) "VAC"
  // SG_ Charger_SOC   : 48|8@1+  (1,0) "%"
  uint32_t current_error_raw {0};
  float    ac_voltage_VAC {0.f};
  uint8_t  charger_soc_pct {0};
  // Helper string expansion for the error code
  String   current_error_text;  // filled by decoder
};

struct DeltaQ_TPDO2_0x28A {
  // SG_ Elapsed_Time : 0|16@1+  (10,0) "s"
  // SG_ Ah_Returned  : 16|32@1+ (0.125,0) "Ah"
  // SG_ Wh_Returned  : 48|16@1+ (0.0625,0) "Wh"
  float elapsed_time_s {0.f};
  float ah_returned_Ah {0.f};
  float wh_returned_Wh {0.f};
};

struct DeltaQ_TPDO1_0x18A {
  // SG_ Charging_Current               : 0|16@1+  (0.00390625,0) "A"
  // SG_ Battery_Voltage                : 16|16@1+ (0.00390625,0) "V"
  // Status nibble/bitfields spread across byte 4..5:
  //  bit34: Charger_Hardware_Shutdown_Status
  //  bit35: Charger_Derating_Status
  //  bit36: AC_Connection_Status
  //  bit37: Charger_Status
  //  bits38..39: Override_Status (2 bits, but table only defines 0/1)
  //  bits40..43: Charge_Indication
  //  bits44..47: Charge_Cycle_Type
  float                          charging_current_A {0.f};
  float                          battery_voltage_V  {0.f};
  ChargerHardwareShutdownStatus  hw_shutdown {ChargerHardwareShutdownStatus::Running};
  ChargerDeratingStatus          derating    {ChargerDeratingStatus::NotDerating};
  ACConnectionStatus             ac_status   {ACConnectionStatus::NoAC};
  ChargerStatus                  charger_status {ChargerStatus::Disabled};
  OverrideStatus                 override_status {OverrideStatus::Disabled};
  ChargeIndication               charge_indication {ChargeIndication::Inactive};
  BattChargeCycleType            charge_cycle_type {BattChargeCycleType::NoActiveCycle};
};

struct NMT_Start_0x000 {
  // SG_ NMT_Command : 0|8@1+  (1,0)
  // SG_ NMT_Node    : 8|8@1+  (1,0)
  NMTCommand command {NMTCommand::Start};
  uint8_t    node_id {0};
};

struct Fault_Register_0x08A {
  // DBC shows 64 bits, but no scaling; keep raw bytes for now
  uint8_t raw[8] {0,0,0,0,0,0,0,0};
};

struct Heartbeat_Response_0x701 {
  HeartbeatState state {HeartbeatState::Operational};
};

struct Heartbeat_0x70A {
  HeartbeatState state {HeartbeatState::Operational};
};

// A variant-like container so one call can return any decoded message.
struct AnyMessage {
  enum class Type {
    None, RPDO2_30A, RPDO1_20A, TPDO3_38A, TPDO2_28A, TPDO1_18A,
    NMT_Start, FaultReg_08A, HB_701, HB_70A
  } type {Type::None};

  DeltaQ_RPDO2_0x30A rpdo2_30a;
  DeltaQ_RPDO1_0x20A rpdo1_20a;
  DeltaQ_TPDO3_0x38A tpdo3_38a;
  DeltaQ_TPDO2_0x28A tpdo2_28a;
  DeltaQ_TPDO1_0x18A tpdo1_18a;
  NMT_Start_0x000    nmt_start;
  Fault_Register_0x08A fault_reg;
  Heartbeat_Response_0x701 hb_701;
  Heartbeat_0x70A          hb_70a;
};

// ---------------------------- Helper toString ------------------------------
String toString(BatteryStatus v);
String toString(ChargerHardwareShutdownStatus v);
String toString(ChargerDeratingStatus v);
String toString(ACConnectionStatus v);
String toString(ChargerStatus v);
String toString(OverrideStatus v);
String toString(ChargeIndication v);
String toString(BattChargeCycleType v);
String toString(NMTCommand v);
String toString(HeartbeatState v);

// Error code -> text (from the DBC VAL_TABLE / VAL_ for Current_Error)
String currentErrorToText(uint32_t code);

} // namespace dbc
