/*
 * ESP32 WebSocket Broadcast Test (standalone, no MSP432 needed)
 * 
 * Purpose: Verify the WebSocket broadcast works end-to-end.
 * Sends a test message every second so you can verify the browser
 * or Python client receives it.
 *
 * Connect to WiFi "ROVER3D", open http://192.168.4.1/
 * You should see "Test: 0", "Test: 1", "Test: 2"... appear.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);
int counter = 0;

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>WS Test</title></head>
<body>
<h2>WebSocket Test</h2>
<p>Status: <span id='s'>Disconnected</span></p>
<div id='log' style='background:#111;color:#0f0;padding:10px;height:400px;overflow:scroll;font-family:monospace'></div>
<script>
const el = document.getElementById('log');
const st = document.getElementById('s');
const wsock = new WebSocket('ws://' + location.hostname + ':81/');
wsock.onopen = () => st.textContent = 'Connected';
wsock.onclose = () => st.textContent = 'Disconnected';
wsock.onmessage = (e) => { el.innerHTML += e.data + '<br>'; el.scrollTop = el.scrollHeight; };
</script>
</body>
</html>
)rawliteral");
}

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    server.on("/", handleRoot);
    server.begin();
    ws.begin();
    ws.onEvent(wsEvent);
    Serial.println("Test server ready");
}

void loop() {
    server.handleClient();
    ws.loop();
    delay(1000);
    String msg = "Test: " + String(counter++);
    ws.broadcastTXT(msg);
    Serial.println(msg);
}