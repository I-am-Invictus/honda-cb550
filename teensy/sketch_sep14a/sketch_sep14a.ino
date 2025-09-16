#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN3 on Teensy 4.1 : CRX3=30, CTX3=31
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can3;

void onRx(const CAN_message_t &msg) {
  Serial.print("ID 0x"); Serial.print(msg.id, HEX);
  Serial.print("  LEN "); Serial.print(msg.len);
  Serial.print("  DATA ");
  for (uint8_t i = 0; i < msg.len; i++) {
    if (msg.buf[i] < 0x10) Serial.print('0');
    Serial.print(msg.buf[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy 4.1 CAN3 listener @ 500k");

  can3.begin();
  can3.setBaudRate(500000);
  // Explicitly set pins (optional; CAN3 defaults to 30/31)
  // Accept all frames by default (don’t add REJECT filters here)
  can3.onReceive(onRx);
  can3.enableMBInterrupts();
}

void loop() {
  can3.events();  // required to dispatch callbacks
}
