/* esp32-sentry.ino
 * ESP32 companion for sentry-target-lock test.
 *
 * Streams ToF distance continuously via UART to MSP432:
 *   D,dist_mm\r\n
 *
 * Also broadcasts to WebSocket dashboard for visualization.
 *
 * Wiring:
 *   ESP32 GPIO16 (RX)  <- MSP432 P3.3 TX
 *   ESP32 GPIO4  (TX)  -> MSP432 P3.2 RX
 *   ESP32 GND          <- MSP432 GND
 *   VL53L1X SDA  -> GPIO21, SCL -> GPIO22, XSHUT -> GPIO17
 *
 * Libraries: Adafruit_VL53L1X, WebSockets by Markus Sattler
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#define XSHUT_PIN   17
#define SDA_PIN     21
#define SCL_PIN     22
#define UART_RX_PIN 16
#define UART_TX_PIN 4

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

/*===========================================================================*/
/* Minimal dashboard for sentry visualization                                */
/*===========================================================================*/

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'><title>Sentry Test</title>
<style>
body{background:#1a1a2e;color:#eee;font-family:monospace;padding:20px}
h1{color:#0ff}
.stat{background:#16213e;padding:15px;border-radius:8px;margin:10px 0}
.stat-val{font-size:36px;color:#0ff}
.lbl{color:#888}
canvas{width:100%;height:200px;background:#0d1117;border:1px solid #333}
</style>
</head>
<body>
<h1>Sentry Target Lock</h1>
<div class=stat><div class=stat-val id=dist>--</div><div class=lbl>Distance mm</div></div>
<div class=stat><div class=stat-val id=pan>--</div><div class=lbl>Pan deg</div></div>
<div class=stat><div class=stat-val id=state>SWEEP</div><div class=lbl>Mode</div></div>
<canvas id=chart></canvas>
<div id=log style="background:#0d1117;padding:10px;height:200px;overflow-y:scroll;font-size:12px;margin-top:10px"></div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
let hist=[], mode='SWEEP';
function resize(){const c=document.getElementById('chart');c.width=c.offsetWidth;c.height=200}
resize();window.onresize=resize;
function drawChart(){
 ctx.fillStyle='#16213e';ctx.fillRect(0,0,ctx.canvas.width,ctx.canvas.height);
 if(hist.length<2)return;
 ctx.strokeStyle='#333';ctx.lineWidth=1;
 for(let y=0;y<=2000;y+=500){let py=ctx.canvas.height-(y/2000)*ctx.canvas.height;ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(ctx.canvas.width,py);ctx.stroke()}
 ctx.strokeStyle='#0ff';ctx.lineWidth=2;ctx.beginPath();
 for(let i=0;i<hist.length;i++){let x=(i/(hist.length-1))*ctx.canvas.width;let y=ctx.canvas.height-(hist[i]/2000)*ctx.canvas.height;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y)}
 ctx.stroke();
 let th=ctx.canvas.height-(300/2000)*ctx.canvas.height;
 ctx.strokeStyle='#f44';ctx.setLineDash([5,5]);ctx.beginPath();ctx.moveTo(0,th);ctx.lineTo(ctx.canvas.width,th);ctx.stroke();ctx.setLineDash([])
}
const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onmessage=e=>{
 const line=e.data;
 document.getElementById('log').innerHTML+=line+'<br>';
 const l=document.getElementById('log');l.scrollTop=l.scrollHeight;
 if(l.children.length>100)l.removeChild(l.firstChild);
 if(line.startsWith('D,')){
   let p=line.split(',');document.getElementById('dist').textContent=p[1];
   hist.push(parseInt(p[1]));if(hist.length>100)hist.shift();drawChart()
 }else if(line.startsWith('P,')){
   let p=line.split(',');document.getElementById('pan').textContent=p[1]
 }else if(line.startsWith('M,')){
   document.getElementById('state').textContent=line.substring(2)
 }
};
</script>
</body>
</html>
)rawliteral");
}

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) { }

/*===========================================================================*/
/* SETUP                                                                     */
/*===========================================================================*/

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 Sentry Test");

    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(50);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(50);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (vl53.begin(0x29, &Wire)) {
        Serial.println("ToF OK");
        vl53.setTimingBudget(20);
        vl53.startRanging();
    } else {
        Serial.println("ToF FAIL");
    }

    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();
    ws.begin();
    ws.onEvent(wsEvent);

    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("Ready");
}

/*===========================================================================*/
/* LOOP - stream ToF distance to MSP432 continuously                       */
/*===========================================================================*/

void send_to_msp(uint16_t dist) {
    Serial1.print("D,");
    Serial1.print(dist);
    Serial1.print("\n");
    Serial.print("TX1: D,");
    Serial.println(dist);
}

void loop() {
    uint16_t dist;
    uint32_t to = 0;

    server.handleClient();
    ws.loop();

    /* Stream distance continuously - no request/response needed */
    while (!vl53.dataReady() && to < 100) {
        delay(1);
        to++;
    }
    if (vl53.dataReady()) {
        dist = vl53.distance();
        vl53.clearInterrupt();
        vl53.startRanging();
        if (dist > 0 && dist < 4000) {
            send_to_msp(dist);
            ws.broadcastTXT("D," + String(dist));
        }
    }
}
