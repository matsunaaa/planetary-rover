#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#define RX_PIN 16

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws = WebSocketsServer(81);

String buffer = "";

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Rover3D</title>
<style>
body{background:#1a1a2e;color:#eee;font-family:Arial,sans-serif;margin:0;padding:20px}
h1{color:#0ff;text-align:center}
.stats{display:flex;justify-content:space-around;margin:20px 0;flex-wrap:wrap}
.stat-box{background:#16213e;padding:20px 40px;border-radius:10px;text-align:center;margin:10px}
.stat-val{font-size:48px;font-weight:bold;color:#0ff}
.stat-lbl{color:#888;font-size:18px}
.ok{color:#0f0}
.wa{color:#f00}
.panel{display:flex;gap:20px;margin-top:20px}
.panel>div{flex:1;background:#16213e;border-radius:10px;padding:20px}
canvas{width:100%;height:300px;background:#000;border:1px solid #333}
#scan3d{height:400px}
.bar-wrap{background:#16213e;border-radius:10px;padding:20px;margin-top:20px}
.dbar{height:60px;background:linear-gradient(90deg,#0ff,#00f);border-radius:5px;transition:width .1s}
.blbl{display:flex;justify-content:space-between;margin-top:10px;color:#888}
#raw{background:#0d1117;padding:10px;height:150px;overflow-y:scroll;border:1px solid #333;font-size:12px;margin-top:20px}
</style>
</head>
<body>
<h1>Rover3D WiFi Dashboard</h1>
<div class=stats>
<div class=stat-box><div class=stat-val id=dist>--</div><div class=stat-lbl>Distance <span style=font-size:24px;color:#888>mm</span></div></div>
<div class=stat-box><div class=stat-val id=st>--</div><div class=stat-lbl>Status</div></div>
<div class=stat-box><div class=stat-val id=rate>--</div><div class=stat-lbl>Samples/sec</div></div>
<div class=stat-box><div class=stat-val id=pts>0</div><div class=stat-lbl>Scan pts</div></div>
</div>
<div class=bar-wrap><div class=dbar id=dbar style=width:0%></div><div class=blbl><span>0</span><span>500</span><span>1000</span><span>1500</span><span>2000mm</span></div></div>
<div class=panel>
<div><h3>3D Scan (color = distance)</h3><canvas id=scan3d></canvas></div>
<div><h3>Distance History</h3><canvas id=chart></canvas></div>
</div>
<div id=raw></div>
<script>
const ctx=document.getElementById('chart').getContext('2d');
const scan=document.getElementById('scan3d').getContext('2d');
let hist=[], scanPts=[], last=0, sc=0;
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
let r=Math.round(255*t), b=Math.round(255*(1-t)), g=Math.round(128*(1-Math.abs(t-0.5)*2));
return `rgb(${r},${g},${b})`;
}

function drawScan3D(){
scan.fillStyle='#0d1117';scan.fillRect(0,0,scan.canvas.width,scan.canvas.height);
if(scanPts.length===0)return;
let minPan=Infinity,maxPan=-Infinity,minTilt=Infinity,maxTilt=-Infinity;
scanPts.forEach(p=>{minPan=Math.min(minPan,p[0]);maxPan=Math.max(maxPan,p[0]);minTilt=Math.min(minTilt,p[1]);maxTilt=Math.max(maxTilt,p[1])});
let pr=minPan===maxPan?1:scan.canvas.width/(maxPan-minPan),tr=minTilt===maxTilt?1:scan.canvas.height/(maxTilt-minTilt);
let r=Math.min(pr,tr);
let ox=scan.canvas.width/2-((minPan+maxPan)/2)*r, oy=scan.canvas.height/2+((minTilt+maxTilt)/2)*r;
let margin=20;
let scale=Math.min((scan.canvas.width-margin*2)/(maxPan-minPan||1),(scan.canvas.height-margin*2)/(maxTilt-minTilt||1));
scale=Math.min(scale,20);
ox=scan.canvas.width/2-((minPan+maxPan)/2)*scale;
oy=scan.canvas.height/2+((minTilt+maxTilt)/2)*scale;
scanPts.forEach(p=>{
let px=ox+p[0]*scale,py=oy-p[1]*scale;
if(px>=0&&px<=scan.canvas.width&&py>=0&&py<=scan.canvas.height){
scan.fillStyle=distColor(p[2]);
scan.beginPath();scan.arc(px,py,3,0,6.28);scan.fill()
}
});
scan.strokeStyle='#333';scan.lineWidth=1;
for(let i=0;i<=scan.canvas.width;i+=40){scan.beginPath();scan.moveTo(i,0);scan.lineTo(i,scan.canvas.height);scan.stroke()}
for(let i=0;i<=scan.canvas.height;i+=40){scan.beginPath();scan.moveTo(0,i);scan.lineTo(scan.canvas.width,i);scan.stroke()}
}

const r=document.getElementById('raw');
const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=()=>document.getElementById('st').textContent='OK';
ws.onclose=()=>document.getElementById('st').textContent='NO';
ws.onmessage=(e)=>{
const line=e.data;
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
scanPts.push([pan,tilt,dist]);if(scanPts.length>2000)scanPts.shift();
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

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, -1);
    
    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    
    server.on("/", handleRoot);
    server.begin();
    
    ws.begin();
    ws.onEvent(wsEvent);
    
    Serial.println("Ready");
}

void loop() {
    server.handleClient();
    ws.loop();
    
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            ws.broadcastTXT(buffer);
            Serial.println(buffer);
            buffer = "";
        } else if (c != '\r') {
            buffer += c;
        }
    }
}