/*
 * ESP32 UART Loopback Test
 * 
 * Purpose: Verify the MSP432 → ESP32 UART connection works.
 * Reads from GPIO16 (Serial1 RX) and echoes everything to USB Serial.
 *
 * Wiring:
 *   ESP32 GPIO16 ← MSP432 P3.3 (TX)
 *   ESP32 GND    ← MSP432 GND
 *
 * If you see data in Serial Monitor, the wiring and UART are good.
 * If you see nothing, check wiring or MSP432 isn't transmitting on P3.3.
 */

void setup() {
  Serial.begin(115200);           // USB debug to PC
  Serial1.begin(115200, SERIAL_8N1, 16, -1);  // UART1: RX=GPIO16, no TX
  Serial.println("\nUART Loopback Test");
  Serial.println("Waiting for data on GPIO16...");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);  // Echo each char to USB
  }
}