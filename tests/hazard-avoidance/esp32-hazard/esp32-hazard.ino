/* esp32-hazard.ino
 * ESP32 companion for hazard-avoidance test.
 *
 * Receives via UART from MSP432:
 *   P,pan,tilt\r\n  - servo position + trigger ToF read
 *   Q\r\n           - query distance only
 *   M,msg\r\n       - mode/status message for dashboard
 *
 * Sends via UART to MSP432:
 *   D,dist_mm\r\n   - ToF distance reading
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
#define DANGER_PIN  26

#define CRITICAL_MM 500

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

/*===========================================================================*/
/* Dashboard HTML                                                            */
/*===========================================================================*/

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'><title>Hazard Test</title>
<style>
body{background:#1a1a2e;color:#eee;font-family:monospace;padding:20px}
h1{color:#0ff}
.stats{display:flex;gap:20px;margin:20px 0}
.stat{background:#16213e;padding:20px;border-radius:8px;flex:1;text-align:center}
.stat-val{font-size:48px;font-weight:bold}
.lbl{color:#888;font-size:14px}
.safe{color:#0f0}
.danger{color:#f44}
canvas{width:100%;height:300px;background:#0d1117;border:1px solid #333;margin-top:10px}
#log{background:#0d1117;padding:10px;height:150px;overflow-y:scroll;font-size:12px;margin-top:10px}
#dbar{height:40px;border-radius:4px;transition:width .1s,background .3s;margin-top:10px}
</style>
</head>
<body>
<h1>Hazard Avoidance Test</h1>
<div class=stats>
<div class=stat><div class=stat-val id=dist>--</div><div class=lbl>Distance mm</div></div>
<div class=stat><div class=stat-val id=pan>--</div><div class=lbl>Pan deg</div></div>
<div class=stat><div class=stat-val id=state id=mode>INIT</div><div class=lbl>Mode</div></div>
</div>
<div id=dbar style="width:0%;background:#0ff"></div>
<div class=lbl style=display:flex;justify-content:space-between;margin-top:4px><span>0mm</span><span id=tv>150mm</span><span>1000+mm</span></div>
<canvas id=chart></canvas>
<div id=log></div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
let hist=[];
function resize(){const c=document.getElementById('chart');c.width=c.offsetWidth;c.height=300}
resize();window.onresize=resize;
function drawChart(){
 ctx.fillStyle='#16213e';ctx.fillRect(0,0,ctx.canvas.width,ctx.canvas.height);
 if(hist.length<2)return;
 ctx.strokeStyle='#333';ctx.lineWidth=1;
 for(let y=0;y<=1000;y+=250){let py=ctx.canvas.height-(y/1000)*ctx.canvas.height;ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(ctx.canvas.width,py);ctx.stroke();ctx.fillStyle='#666';ctx.fillText(y+'mm',5,py-5)}
 let th=ctx.canvas.height-(150/1000)*ctx.canvas.height;
 ctx.strokeStyle='#f44';ctx.setLineDash([8,4]);ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(0,th);ctx.lineTo(ctx.canvas.width,th);ctx.stroke();ctx.setLineDash([]);
 ctx.fillStyle='#f44';ctx.font='11px monospace';ctx.fillText('CRITICAL',ctx.canvas.width-80,th-5);
 ctx.strokeStyle='#0ff';ctx.lineWidth=2;ctx.beginPath();
 for(let i=0;i<hist.length;i++){let x=(i/(hist.length-1))*ctx.canvas.width;let y=ctx.canvas.height-(hist[i]/1000)*ctx.canvas.height;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y)}
 ctx.stroke();ctx.lineTo(ctx.canvas.width,ctx.canvas.height);ctx.lineTo(0,ctx.canvas.height);ctx.closePath();ctx.fillStyle='rgba(0,255,255,0.08)';ctx.fill()
}
const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onmessage=e=>{
 const line=e.data;
 const l=document.getElementById('log');l.innerHTML+=line+'<br>';l.scrollTop=l.scrollHeight;
 if(l.children.length>100)l.removeChild(l.firstChild);
 if(line.startsWith('D,')){
   let p=line.split(',');let d=parseInt(p[1]);
   document.getElementById('dist').textContent=d;
   let pct=Math.min(100,(d/1000)*100);
   document.getElementById('dbar').style.width=pct+'%';
   if(d<150){document.getElementById('dbar').style.background='#f44';document.getElementById('dist').className='stat-val danger'}
   else{document.getElementById('dbar').style.background='#0ff';document.getElementById('dist').className='stat-val safe'}
   hist.push(d);if(hist.length>200)hist.shift();drawChart()
 }else if(line.startsWith('P,')){
   document.getElementById('pan').textContent=line.split(',')[1]
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
    Serial.println("\nESP32 Hazard Test");

    pinMode(DANGER_PIN, OUTPUT);
    digitalWrite(DANGER_PIN, LOW);

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

    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, -1);
    Serial.println("Ready");
}

/*===========================================================================*/
/* LOOP                                                                      */
/*===========================================================================*/

String lastMode = "INIT";
uint32_t lastModeTime = 0;

void loop() {
    static String buf;
    uint16_t dist;
    uint32_t to = 0;

    server.handleClient();
    ws.loop();

    /* Forward P, (pan) and M, (mode) messages from MSP432 to dashboard */
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            buf.trim();
            if (buf.length() > 0) {
                if (buf.startsWith("M,")) {
                    lastMode = buf;
                }
                if (buf.startsWith("P,") || buf.startsWith("M,")) {
                    ws.broadcastTXT(buf);
                }
            }
            buf = "";
        } else if (c != '\r') {
            buf += c;
        }
    }

    /* Re-broadcast current mode every 500ms so dashboard stays in sync
     * even if a browser connected after the mode was first sent */
    if (millis() - lastModeTime > 500) {
        lastModeTime = millis();
        ws.broadcastTXT(lastMode);
    }

    /* Stream distance continuously, drive DANGER_PIN HIGH when obstacle
     * is within CRITICAL_MM */
    while (!vl53.dataReady() && to < 100) {
        delay(1);
        to++;
    }
    if (vl53.dataReady()) {
        dist = vl53.distance();
        vl53.clearInterrupt();
        vl53.startRanging();
        if (dist > 0 && dist < 4000) {
            digitalWrite(DANGER_PIN, (dist < CRITICAL_MM) ? HIGH : LOW);
            ws.broadcastTXT("D," + String(dist));
        }
    }
}
