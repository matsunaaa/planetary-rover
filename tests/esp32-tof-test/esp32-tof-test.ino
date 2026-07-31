/* esp32-tof-test.ino
 * ESP32 reads VL53L1X ToF, receives servo position from MSP432 via UART,
 * broadcasts combined 3D scan data via WebSocket dashboard.
 *
 * Data flow:
 *   MSP432 UART (P3.3 TX) -> ESP32 GPIO16: "P,pan,tilt\n"
 *   ESP32 reads ToF -> broadcasts: "S,pan,tilt,dist" + "D,dist,status"
 *
 * Wiring:
 *   VL53L1X SDA   -> GPIO21, SCL -> GPIO22, XSHUT -> GPIO17
 *   ESP32 GPIO16  <- MSP432 P3.3 TX
 *   ESP32 GND     <- MSP432 GND
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
#define RX_PIN      16

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

/*===========================================================================*/
/* Dashboard HTML with 3D scan visualization                                */
/*===========================================================================*/

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Rover3D Scan</title>
<style>
body{background:#1a1a2e;color:#eee;font-family:Arial,sans-serif;margin:0;padding:20px}
h1{color:#0ff;text-align:center}
.stats{display:flex;justify-content:space-around;margin:20px 0;flex-wrap:wrap}
.stat-box{background:#16213e;padding:20px 40px;border-radius:10px;text-align:center;margin:10px}
.stat-val{font-size:48px;font-weight:bold;color:#0ff}
.stat-lbl{color:#888;font-size:18px}
.ok{color:#0f0}
.wa{color:#f00}
.bar-wrap{background:#16213e;border-radius:10px;padding:20px;margin-top:20px}
.dbar{height:60px;background:linear-gradient(90deg,#0ff,#00f);border-radius:5px;transition:width .1s}
.blbl{display:flex;justify-content:space-between;margin-top:10px;color:#888}
.panel{display:flex;gap:20px;margin-top:20px}
.panel>div{flex:1;background:#16213e;border-radius:10px;padding:20px}
canvas{width:100%;height:300px;background:#000;border:1px solid #333}
#scan3d{height:400px}
#raw{background:#0d1117;padding:10px;height:150px;overflow-y:scroll;border:1px solid #333;font-size:12px;margin-top:20px}
</style>
</head>
<body>
<h1>Rover3D 3D Scan</h1>
<div class=stats>
<div class=stat-box><div class=stat-val id=dist>--</div><div class=stat-lbl>Distance <span style=font-size:24px;color:#888>mm</span></div></div>
<div class=stat-box><div class=stat-val id=st>--</div><div class=stat-lbl>Status</div></div>
<div class=stat-box><div class=stat-val id=rate>--</div><div class=stat-lbl>Samples/sec</div></div>
<div class=stat-box><div class=stat-val id=pts>0</div><div class=stat-lbl>Scan pts</div></div>
</div>
<div class=bar-wrap><div class=dbar id=dbar style=width:0%></div><div class=blbl><span>0</span><span>500</span><span>1000</span><span>1500</span><span>2000mm</span></div></div>
<div class=panel>
<div><h3>3D Point Cloud</h3>
<div style=margin-bottom:8px>
<button id=btnAuto class=cb>Auto-rotate</button>
<button id=btnReset class=cb>Reset View</button>
</div>
<canvas id=scan3d></canvas></div>
<div><h3>Distance History</h3><canvas id=chart></canvas></div>
</div>
<div id=raw></div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
const scan=document.getElementById('scan3d');
let hist=[], scanPts=[], last=0, sc=0;
let rotX=-0.3, rotY=0.5, zoom=1, mx=0, my=0, dragging=false, autoRot=false, autoTimer;

function resize(){const c=document.getElementById('chart');c.width=c.offsetWidth;c.height=300}
function resizeScan(){const c=document.getElementById('scan3d');c.width=c.offsetWidth;c.height=c.offsetWidth}
resize();resizeScan();window.onresize=()=>{resize();resizeScan()};

function drawChart(){
ctx.fillStyle='#16213e';ctx.fillRect(0,0,ctx.canvas.width,ctx.canvas.height);
if(hist.length<2)return;
ctx.strokeStyle='#333';ctx.lineWidth=1;
for(let y=0;y<=2000;y+=500){let py=ctx.canvas.height-(y/2000)*ctx.canvas.height;ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(ctx.canvas.width,py);ctx.stroke();ctx.fillStyle='#666';ctx.fillText(y+'mm',5,py-5)}
ctx.strokeStyle='#0ff';ctx.lineWidth=2;ctx.beginPath();
for(let i=0;i<hist.length;i++){let x=(i/(hist.length-1))*ctx.canvas.width;let y=ctx.canvas.height-(hist[i]/2000)*ctx.canvas.height;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y)}
ctx.stroke();ctx.lineTo(ctx.canvas.width,ctx.canvas.height);ctx.lineTo(0,ctx.canvas.height);ctx.closePath();ctx.fillStyle='rgba(0,255,255,0.1)';ctx.fill()
}

function distColor(d){
let t=Math.min(1,Math.max(0,d/2000));
let r=Math.round(255*t),b=Math.round(255*(1-t)),g=Math.round(128*(1-Math.abs(t-0.5)*2));
return 'rgb('+r+','+g+','+b+')';
}

function to3D(pan,tilt,dist){
let pr=(pan-45)*Math.PI/180, tr=(tilt-90)*Math.PI/180;
return {x:dist*Math.sin(pr)*Math.cos(tr), y:dist*Math.cos(pr)*Math.cos(tr), z:dist*Math.sin(tr)};
}

function drawScan3D(){
let w=scan.width, h=scan.height, c3=scan.getContext('2d');
c3.fillStyle='#0d1117';c3.fillRect(0,0,w,h);
if(scanPts.length===0)return;
let cosX=Math.cos(rotX), sinX=Math.sin(rotX), cosY=Math.cos(rotY), sinY=Math.sin(rotY);
function proj(x,y,z){
  let x1=x*cosY-z*sinY, z1=x*sinY+z*cosY;
  let y1=y*cosX-z1*sinX, z2=y*sinX+z1*cosX;
  let s=600/(600+z2*zoom);
  return {sx:w/2+x1*s*zoom, sy:h/2-y1*s*zoom, dep:z2, sc:s};
}

// Data bounds for axis/grid sizing
let maxR=0, minX=1e9, maxX=-1e9, minY=1e9, maxY=-1e9, minZ=1e9, maxZ=-1e9;
for(let p of scanPts){
  let pt=to3D(p[0],p[1],p[2]);
  if(p[2]>maxR)maxR=p[2];
  if(pt.x<minX)minX=pt.x;if(pt.x>maxX)maxX=pt.x;
  if(pt.y<minY)minY=pt.y;if(pt.y>maxY)maxY=pt.y;
  if(pt.z<minZ)minZ=pt.z;if(pt.z>maxZ)maxZ=pt.z;
}
let aLen=Math.max(200,maxR*0.6);

// Ground grid (z=0 plane)
let gs=aLen*0.8, step=50;
c3.strokeStyle='rgba(255,255,255,0.06)';c3.lineWidth=1;
for(let g=-gs;g<=gs;g+=step){
  let a=proj(g,-gs,0), b=proj(g,gs,0);
  if(a.dep<0&&b.dep<0)continue;
  c3.beginPath();c3.moveTo(a.sx,a.sy);c3.lineTo(b.sx,b.sy);c3.stroke();
  a=proj(-gs,g,0);b=proj(gs,g,0);
  if(a.dep<0&&b.dep<0)continue;
  c3.beginPath();c3.moveTo(a.sx,a.sy);c3.lineTo(b.sx,b.sy);c3.stroke();
}

// Points sorted by depth (far first)
let pp=[];
for(let p of scanPts){
  let pt=to3D(p[0],p[1],p[2]), pj=proj(pt.x,pt.y,pt.z);
  pp.push({sx:pj.sx,sy:pj.sy,sz:pj.sc,d:p[2],z:pj.dep});
}
pp.sort((a,b)=>b.z-a.z);
for(let p of pp){
  c3.fillStyle=distColor(p.d);
  c3.beginPath();c3.arc(p.sx,p.sy,Math.max(1,2*p.sz*zoom),0,Math.PI*2);c3.fill();
}

// Axes with arrowheads
function drawAxis(x1,y1,z1,x2,y2,z2,color,label){
  let a=proj(x1,y1,z1), b=proj(x2,y2,z2);
  if(a.dep<-100||b.dep<-100)return;
  c3.strokeStyle=color;c3.lineWidth=2;
  c3.beginPath();c3.moveTo(a.sx,a.sy);c3.lineTo(b.sx,b.sy);c3.stroke();
  dx=b.sx-a.sx;dy=b.sy-a.sy;len=Math.sqrt(dx*dx+dy*dy);
  if(len>8){
    let ux=dx/len, uy=dy/len, as=10;
    c3.fillStyle=color;
    c3.beginPath();c3.moveTo(b.sx,b.sy);
    c3.lineTo(b.sx-ux*as-uy*as*0.4,b.sy-uy*as+ux*as*0.4);
    c3.lineTo(b.sx-ux*as+uy*as*0.4,b.sy-uy*as-ux*as*0.4);
    c3.closePath();c3.fill();
    c3.font='bold 13px monospace';c3.textAlign='left';
    c3.fillText(label,b.sx+6,b.sy+4);
  }
}
drawAxis(0,0,0,aLen,0,0,'#ff4444','X (Right)');
drawAxis(0,0,0,0,aLen,0,'#44ff44','Y (Forward)');
drawAxis(0,0,0,0,0,aLen,'#4488ff','Z (Up)');

// Color legend bar
let lx=10, ly=h-35, lw=Math.min(200,w-20), lh=12;
let grd=c3.createLinearGradient(lx,ly,lx+lw,ly);
grd.addColorStop(0,'#0044ff');grd.addColorStop(0.5,'#80ff80');grd.addColorStop(1,'#ff4400');
c3.fillStyle=grd;c3.fillRect(lx,ly,lw,lh);
c3.strokeStyle='#888';c3.lineWidth=1;c3.strokeRect(lx,ly,lw,lh);
c3.fillStyle='#aaa';c3.font='10px monospace';
c3.textAlign='left';c3.fillText('0mm',lx,ly-4);
c3.textAlign='right';c3.fillText('2000mm',lx+lw,ly-4);

// Stats overlay
c3.fillStyle='rgba(255,255,255,0.35)';c3.font='11px monospace';c3.textAlign='right';
c3.fillText(scanPts.length+' pts  Drag=rotate  Wheel=zoom',w-10,16);
}

function resetView(){rotX=-0.3;rotY=0.5;zoom=1;drawScan3D();}
function toggleAuto(){
  autoRot=!autoRot;
  if(autoRot){autoTimer=setInterval(()=>{if(autoRot){rotY+=0.008;drawScan3D()}},30)}
  else{clearInterval(autoTimer)}
}
document.getElementById('btnAuto').onclick=toggleAuto;
document.getElementById('btnReset').onclick=resetView;

scan.addEventListener('mousedown',e=>{dragging=true;mx=e.clientX;my=e.clientY;});
window.addEventListener('mousemove',e=>{if(!dragging)return;let dx=e.clientX-mx,dy=e.clientY-my;rotY+=dx*0.01;rotX+=dy*0.01;mx=e.clientX;my=e.clientY;drawScan3D();});
window.addEventListener('mouseup',()=>{dragging=false;});
scan.addEventListener('wheel',e=>{zoom*=e.deltaY>0?0.9:1.1;zoom=Math.max(0.1,Math.min(10,zoom));drawScan3D();e.preventDefault();});

const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=()=>document.getElementById('st').textContent='OK';
ws.onclose=()=>document.getElementById('st').textContent='NO';
ws.onmessage=(e)=>{
const line=e.data;
const r=document.getElementById('raw');
r.innerHTML+=line+'<br>';r.scrollTop=r.scrollHeight;
if(r.children.length>200)r.removeChild(r.firstChild);
if(line.startsWith('D,')){
let p=line.split(',');
if(p.length>=3){
let d=parseInt(p[1]),s=parseInt(p[2]);
document.getElementById('dist').textContent=d;
document.getElementById('st').textContent=s===1?'OK':'WAIT';
document.getElementById('st').className='stat-val '+(s===1?'ok':'wa');
let pct=Math.min(100,(d/2000)*100);
document.getElementById('dbar').style.width=pct+'%';
hist.push(d);if(hist.length>200)hist.shift();
drawChart();sc++;let n=Date.now();if(n-last>1000){document.getElementById('rate').textContent=sc;sc=0;last=n}
}
}else if(line.startsWith('S,')){
let p=line.split(',');
if(p.length>=4){
let pan=parseInt(p[1]),tilt=parseInt(p[2]),dist=parseInt(p[3]);
if(dist>20&&dist<2000){scanPts.push([pan,tilt,dist]);if(scanPts.length>5000)scanPts.shift();}
document.getElementById('pts').textContent=scanPts.length;
drawScan3D()
}
}
};
</script>
</body>
</html>
)rawliteral");
}

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {}

/*===========================================================================*/
/* SETUP                                                                     */
/*===========================================================================*/

void setup() {
    Serial.begin(115200);
    Serial.println("\nRover3D ESP32 Bridge + ToF");

    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(50);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(50);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (vl53.begin(0x29, &Wire)) {
        Serial.println("ToF init OK");
        vl53.startRanging();
    } else {
        Serial.println("ToF init FAIL");
    }

    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();
    ws.begin();
    ws.onEvent(wsEvent);

    Serial1.begin(115200, SERIAL_8N1, RX_PIN, -1);
    Serial.println("Ready");
}

/*===========================================================================*/
/* LOOP                                                                      */
/*===========================================================================*/

void loop() {
    server.handleClient();
    ws.loop();

    /* Read all available UART lines from MSP432 */
    static String uartBuf;
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            uartBuf.trim();
            if (uartBuf.length() > 0) {
                Serial.println("RX: " + uartBuf);

                    if (uartBuf.startsWith("P,")) {
                        int comma = uartBuf.indexOf(',', 2);
                        if (comma > 0) {
                            int pan = uartBuf.substring(2, comma).toInt();
                            int tilt = uartBuf.substring(comma + 1).toInt();

                            uint16_t dist = 0;
                            if (vl53.dataReady()) {
                                dist = vl53.distance();
                                vl53.clearInterrupt();
                                vl53.startRanging();
                            }

                            if (dist > 20 && dist < 2000) {
                                ws.broadcastTXT("S," + String(pan) + "," + String(tilt) + "," + String(dist));
                                ws.broadcastTXT("D," + String(dist) + ",1");
                            } else {
                                ws.broadcastTXT("D," + String(dist) + ",0");
                            }
                    }
                }
            }
            uartBuf = "";
        } else if (c != '\r') {
            uartBuf += c;
        }
    }
}
