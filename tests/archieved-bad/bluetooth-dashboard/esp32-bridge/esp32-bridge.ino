/*
 * ESP32 Bluetooth SPP Bridge
 * Receives from MSP432 via UART2, forwards over Bluetooth to PC
 *
 * Wiring:
 *   ESP32 GPIO16 (UART2 RX)  ← MSP432 P3.3 (TX)
 *   ESP32 GPIO17 (UART2 TX)  → MSP432 P3.2 (RX)  (optional)
 *   GND                      ↔ GND
 *
 * Pair with "ROVER3D" from PC, then connect dashboard.py to the Bluetooth COM port.
 */

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);          // USB debug to PC
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // UART2: RX=GPIO16, TX=GPIO17

  SerialBT.begin("ROVER3D");     // Bluetooth device name
  Serial.println("\nESP32 ROVER3D Bridge Ready");
  Serial.println("Pair with 'ROVER3D' via Bluetooth");
}

void loop() {
  /* MSP432 → Bluetooth (forward sensor data to PC dashboard) */
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      SerialBT.println(line);
      Serial.println(line);  // Also echo to USB debug
    }
  }

  /* Bluetooth → MSP432 (optional: commands from dashboard) */
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial2.write(c);
    Serial.write(c);  // Echo to USB debug
  }
}