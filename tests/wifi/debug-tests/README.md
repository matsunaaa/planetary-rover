# WiFi Debug Tests

Use these to isolate which layer is failing.

## Test 1: ESP32 UART Loopback
**File:** `esp32-uart-loopback.ino`

Tests if the ESP32 can receive data from the MSP432 via UART.

1. Flash to ESP32
2. Open Serial Monitor (115200 baud)
3. If MSP432 is sending on P3.3 → GPIO16, you'll see data echo in Serial Monitor
4. If nothing appears: wiring issue or MSP432 not transmitting on P3.3

## Test 2: ESP32 WebSocket Broadcast
**File:** `esp32-ws-test.ino`

Tests if WebSocket broadcasting works (no MSP432 needed).

1. Flash to ESP32
2. Connect PC to WiFi "ROVER3D" (password: 12345678)
3. Open browser to http://192.168.4.1/
4. You should see "Test: 0", "Test: 1"... appear every second
5. If not: WebSocket library or WiFi AP issue

## Test 3: Python WebSocket Client
**File:** `ws-client-test.py`

Tests WebSocket from Python (bypasses browser).

1. `pip install websocket-client`
2. Connect PC to WiFi "ROVER3D"
3. Run: `python ws-client-test.py`
4. Should print live data from ESP32

## Likely Issue

The existing `dashboard.py` reads from a **serial COM port**, but the ESP32 sends data over **WiFi WebSocket**. These are two completely different pipelines. The dashboard needs to either:
- Connect to the ESP32's WiFi and read its WebSocket, OR
- The MSP432 sends data via USB serial (COM6) directly to the dashboard