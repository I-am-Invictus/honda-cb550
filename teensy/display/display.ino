#include "Free_Fonts.h"
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);  // sprite for double buffering

float mph, voltage, current, power, soc, rpm, Btemp, Mtemp;

HardwareSerial SerialPort(2);  // use UART2

void setup() {
  Serial.begin(115200); // USB debug still works

  // UART2 on IO35 (RX), IO27 (TX, unused)
  SerialPort.begin(115200, SERIAL_8N1, 35, 27);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // create sprite same size as screen, 16-bit color
  spr.setColorDepth(8);
  spr.createSprite(320, 240);

  drawStaticLabels();
}

void loop() {
  if (SerialPort.available()) {
    String line = SerialPort.readStringUntil('\n');
    Serial.println("RX line: " + line);   // debug to USB
    parseData(line);
    updateDisplay();
  }
}

void drawStaticLabels() {
  spr.fillSprite(TFT_BLACK);

  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setFreeFont(&FreeSans9pt7b);

  int y = 150;  
  int dy = 25;

  spr.setCursor(10, y);       spr.print("Voltage:");
  spr.setCursor(10, y+dy);    spr.print("Current:");
  spr.setCursor(10, y+2*dy);  spr.print("Power:");
  spr.setCursor(10, y+3*dy);  spr.print("SOC:");

  spr.setCursor(180, y);      spr.print("RPM:");
  spr.setCursor(180, y+dy);   spr.print("BT:");
  spr.setCursor(180, y+2*dy); spr.print("MT:");

  spr.pushSprite(0, 0); // draw initial static screen
}


void updateDisplay() {
  spr.fillSprite(TFT_BLACK);  // clear sprite

  // MPH
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  spr.setFreeFont(&FreeSansBold24pt7b);
  spr.setCursor(40, 70);
  spr.printf("%.1f MPH", mph);

  int y = 150;
  int dy = 25;

  // --- Labels (white) ---
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.setFreeFont(&FreeSans9pt7b);

  spr.setCursor(10, y);       spr.print("Voltage:");
  spr.setCursor(10, y+dy);    spr.print("Current:");
  spr.setCursor(10, y+2*dy);  spr.print("Power:");
  spr.setCursor(10, y+3*dy);  spr.print("SOC:");

  spr.setCursor(180, y);      spr.print("RPM:");
  spr.setCursor(180, y+dy);   spr.print("BT:");
  spr.setCursor(180, y+2*dy); spr.print("MT:");

  // --- Values (cyan) ---
  spr.setTextColor(TFT_CYAN, TFT_BLACK);
  spr.setFreeFont(&FreeSans12pt7b);

  spr.setCursor(90, y);       spr.printf("%.1f V", voltage);
  spr.setCursor(90, y+dy);    spr.printf("%.1f A", current);
  spr.setCursor(90, y+2*dy);  spr.printf("%.1f kW", power/1000.0);
  spr.setCursor(90, y+3*dy);  spr.printf("%.0f %%", soc);

  spr.setCursor(240, y);       spr.printf("%.0f", rpm);
  spr.setCursor(240, y+dy);    spr.printf("%.1f C", Btemp);
  spr.setCursor(240, y+2*dy);  spr.printf("%.1f C", Mtemp);

  spr.pushSprite(0, 0);
}


void parseData(String line) {
  line.trim();
  if (line.length() == 0) return;

  int idx = -1, last = 0;
  mph     = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  voltage = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  current = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  power   = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  soc     = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  rpm     = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  Btemp   = line.substring(last, idx = line.indexOf(',', last)).toFloat(); last = idx+1;
  Mtemp   = line.substring(last).toFloat();
}
