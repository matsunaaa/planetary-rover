/* uart-loopback-test.ino
 * ESP32-only test: transmit D,123 on GPIO4 and receive on GPIO16 (loopback).
 * If you see "got: D,123" in the serial monitor, the ESP32 TX pin works.
 */

#define UART_RX_PIN 16
#define UART_TX_PIN 4

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nUART Loopback Test");
    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("Sending on GPIO4, listening on GPIO16...");
}

void loop() {
    Serial1.println("D,123");
    delay(500);

    while (Serial1.available()) {
        String s = Serial1.readStringUntil('\n');
        Serial.println("got: " + s);
    }
}
