/* esp32-rover3d.ino
 * Receives servo pos + rover pos from MSP432, reads ToF, 
 * computes world coordinates, broadcasts to dashboard.
 *
 * Protocol from MSP432:
 *   P,pan,tilt     - servo position, trigger ToF read
 *   R,x,y,heading  - rover position update (x,y in mm, heading in deg)
 *
 * Protocol to dashboard:
 *   S,pan,tilt,dist          - raw scan point
 *   W,world_x,world_y,world_z - world coordinate point
 *   R,x,y,heading            - rover position
 *   D,dist,status            - distance display
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <math.h>

#define XSHUT_PIN   17
#define SDA_PIN     21
#define SCL_PIN     22
#define RX_PIN      16
#define DANGER_PIN  26

#define CRITICAL_MM 500

const char* ssid = "ROVER3D";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer ws(81);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

/* Rover state */
float rover_x = 0;       /* mm */
float rover_y = 0;       /* mm */
float rover_heading = 0; /* degrees */

/* Latest streamed distance for danger pin + mapping */
static uint16_t last_dist = 0;

#define DEG_TO_RAD 0.0174533f
#define PAN_CENTER 45
#define TILT_CENTER 90

/*===========================================================================*/
/* Coordinate transform: servo angles + distance -> world XYZ               */
/*===========================================================================*/

void compute_world_point(int pan, int tilt, int dist, 
                         int *wx, int *wy, int *wz) {
    /* Center the angles */
    float pan_centered = (float)(pan - PAN_CENTER);
    float tilt_centered = (float)(tilt - TILT_CENTER);

    float pan_rad = pan_centered * DEG_TO_RAD;
    float tilt_rad = tilt_centered * DEG_TO_RAD;
    float head_rad = rover_heading * DEG_TO_RAD;

    /* Local coordinates (relative to rover) */
    float local_x = dist * sin(pan_rad) * cos(tilt_rad);
    float local_y = dist * cos(pan_rad) * cos(tilt_rad);
    float local_z = dist * sin(tilt_rad);

    /* Rotate by rover heading */
    float world_x = rover_x + local_x * cos(head_rad) - local_y * sin(head_rad);
    float world_y = rover_y + local_x * sin(head_rad) + local_y * cos(head_rad);
    float world_z = local_z;

    *wx = (int)world_x;
    *wy = (int)world_y;
    *wz = (int)world_z;
}

/*===========================================================================*/
/* Dashboard HTML                                                            */
/*===========================================================================*/

void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<title>Rover3D Map</title>
<style>
*{box-sizing:border-box}
body{background:#1a1a2e;color:#eee;font-family:monospace;margin:0;padding:10px}
h1{color:#0ff;text-align:center;margin:10px 0}
.top{display:flex;gap:10px;margin-bottom:10px}
.stat{background:#16213e;padding:15px;border-radius:8px;text-align:center;flex:1}
.stat-val{font-size:28px;color:#0ff;font-weight:bold}
.stat-lbl{color:#888;font-size:12px}
.main{display:flex;gap:10px;height:calc(100vh - 200px)}
.panel{background:#16213e;border-radius:8px;padding:10px;flex:1;display:flex;flex-direction:column}
canvas{background:#0d1117;border:1px solid #333;flex:1}
.log{background:#0d1117;padding:8px;height:100px;overflow-y:scroll;font-size:11px;border:1px solid #333;margin-top:10px}
.ok{color:#0f0}.warn{color:#f00}
button{background:#0ff;color:#000;border:none;padding:6px 12px;margin:2px;cursor:pointer;font-family:monospace}
button:hover{background:#0aa}
</style>
</head>
<body>
<h1>Rover3D Autonomous Mapping</h1>

<div class="top">
<div class="stat"><div class="stat-val" id="dist">--</div><div class="stat-lbl">Distance mm</div></div>
<div class="stat"><div class="stat-val" id="rx">0</div><div class="stat-lbl">Rover X mm</div></div>
<div class="stat"><div class="stat-val" id="ry">0</div><div class="stat-lbl">Rover Y mm</div></div>
<div class="stat"><div class="stat-val" id="rh">0</div><div class="stat-lbl">Heading deg</div></div>
<div class="stat"><div class="stat-val" id="pts">0</div><div class="stat-lbl">Points</div></div>
<div class="stat"><div class="stat-val" id="st">--</div><div class="stat-lbl">Status</div></div>
</div>

<div style="margin-bottom:10px">
<button onclick="clearPts()">Clear Points</button>
<button onclick="toggleAuto()">Auto-rotate</button>
<button onclick="resetView()">Reset View</button>
<button onclick="togglePath()">Toggle Path</button>
<button onclick="downloadPly()">Download PLY</button>
</div>

<div class="main">
<div class="panel"><h3>Top-Down Map (X-Y)</h3><canvas id="map2d"></canvas></div>
<div class="panel"><h3>3D Point Cloud</h3><canvas id="map3d"></canvas></div>
</div>

<div class="log" id="log"></div>

<script>
let pts=[], path=[], roverX=0, roverY=0, roverH=0;
let rotX=-0.4, rotY=0.6, zoom=0.3, dragging=false, mx=0, my=0;
let autoRot=false, showPath=true;

const c2d=document.getElementById('map2d');
const c3d=document.getElementById('map3d');
const ctx2=c2d.getContext('2d');
const ctx3=c3d.getContext('2d');

function resize(){
  c2d.width=c2d.offsetWidth;c2d.height=c2d.offsetHeight;
  c3d.width=c3d.offsetWidth;c3d.height=c3d.offsetHeight;
}
resize();window.onresize=resize;

function log(s){
  const l=document.getElementById('log');
  l.innerHTML+=s+'<br>';
  l.scrollTop=l.scrollHeight;
  if(l.children.length>100)l.removeChild(l.firstChild);
}

function clearPts(){pts=[];path=[];log('Cleared');}
function toggleAuto(){autoRot=!autoRot;}
function togglePath(){showPath=!showPath;}
function resetView(){rotX=-0.4;rotY=0.6;zoom=0.3;}

function downloadPly(){
  if(pts.length===0){log('No points to download');return;}
  let head='ply\nformat ascii 1.0\n'
    +'element vertex '+pts.length+'\n'
    +'property float x\nproperty float y\nproperty float z\n'
    +'property uchar red\nproperty uchar green\nproperty uchar blue\n'
    +'end_header\n';
  let body='';
  for(let p of pts){
    let d=Math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
    let t=Math.min(1,d/2000);
    let r=Math.round(50+t*200),g=Math.round(200-t*150),b=Math.round(255-t*200);
    body+=p.x+' '+p.y+' '+p.z+' '+r+' '+g+' '+b+'\n';
  }
  const blob=new Blob([head+body],{type:'text/plain'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(blob);
  a.download='rover3d-cloud.ply';
  a.click();
  URL.revokeObjectURL(a.href);
  log('Exported '+pts.length+' points to rover3d-cloud.ply');
}

function distColor(d){
  let t=Math.min(1,d/2000);
  return `rgb(${Math.round(50+t*200)},${Math.round(200-t*150)},${Math.round(255-t*200)})`;
}

function draw2D(){
  const w=c2d.width, h=c2d.height, cx=w/2, cy=h/2;
  const scale=0.15;
  
  ctx2.fillStyle='#0d1117';ctx2.fillRect(0,0,w,h);
  
  // Grid
  ctx2.strokeStyle='#222';ctx2.lineWidth=1;
  for(let g=-2000;g<=2000;g+=200){
    ctx2.beginPath();
    ctx2.moveTo(cx+g*scale,0);ctx2.lineTo(cx+g*scale,h);
    ctx2.moveTo(0,cy-g*scale);ctx2.lineTo(w,cy-g*scale);
    ctx2.stroke();
  }
  
  // Axes
  ctx2.strokeStyle='#444';ctx2.lineWidth=2;
  ctx2.beginPath();ctx2.moveTo(cx,0);ctx2.lineTo(cx,h);ctx2.stroke();
  ctx2.beginPath();ctx2.moveTo(0,cy);ctx2.lineTo(w,cy);ctx2.stroke();
  
  // Path
  if(showPath && path.length>1){
    ctx2.strokeStyle='#ff0';ctx2.lineWidth=2;ctx2.beginPath();
    ctx2.moveTo(cx+path[0][0]*scale, cy-path[0][1]*scale);
    for(let i=1;i<path.length;i++){
      ctx2.lineTo(cx+path[i][0]*scale, cy-path[i][1]*scale);
    }
    ctx2.stroke();
  }
  
  // Points
  for(let p of pts){
    ctx2.fillStyle=distColor(Math.sqrt(p.x*p.x+p.y*p.y));
    ctx2.beginPath();
    ctx2.arc(cx+p.x*scale, cy-p.y*scale, 2, 0, Math.PI*2);
    ctx2.fill();
  }
  
  // Rover
  let rx=cx+roverX*scale, ry=cy-roverY*scale;
  let hr=-roverH*Math.PI/180;
  ctx2.fillStyle='#f00';
  ctx2.beginPath();
  ctx2.moveTo(rx+Math.sin(hr)*15, ry-Math.cos(hr)*15);
  ctx2.lineTo(rx+Math.sin(hr+2.5)*8, ry-Math.cos(hr+2.5)*8);
  ctx2.lineTo(rx+Math.sin(hr-2.5)*8, ry-Math.cos(hr-2.5)*8);
  ctx2.closePath();ctx2.fill();
}

function draw3D(){
  const w=c3d.width, h=c3d.height;
  ctx3.fillStyle='#0d1117';ctx3.fillRect(0,0,w,h);
  
  if(autoRot)rotY+=0.005;
  
  let cosX=Math.cos(rotX), sinX=Math.sin(rotX);
  let cosY=Math.cos(rotY), sinY=Math.sin(rotY);
  
  function proj(x,y,z){
    let x1=x*cosY-z*sinY, z1=x*sinY+z*cosY;
    let y1=y*cosX-z1*sinX, z2=y*sinX+z1*cosX;
    let s=400/(400+z2*zoom);
    return {sx:w/2+x1*s*zoom*0.3, sy:h/2-y1*s*zoom*0.3, z:z2, s:s};
  }
  
  // Grid
  ctx3.strokeStyle='rgba(255,255,255,0.05)';
  for(let g=-1000;g<=1000;g+=200){
    let a=proj(g,-1000,0), b=proj(g,1000,0);
    ctx3.beginPath();ctx3.moveTo(a.sx,a.sy);ctx3.lineTo(b.sx,b.sy);ctx3.stroke();
    a=proj(-1000,g,0);b=proj(1000,g,0);
    ctx3.beginPath();ctx3.moveTo(a.sx,a.sy);ctx3.lineTo(b.sx,b.sy);ctx3.stroke();
  }
  
  // Axes
  let axLen=300;
  [{c:'#f44',l:'X',v:[axLen,0,0]},{c:'#4f4',l:'Y',v:[0,axLen,0]},{c:'#44f',l:'Z',v:[0,0,axLen]}].forEach(a=>{
    let o=proj(0,0,0), e=proj(a.v[0],a.v[1],a.v[2]);
    ctx3.strokeStyle=a.c;ctx3.lineWidth=2;
    ctx3.beginPath();ctx3.moveTo(o.sx,o.sy);ctx3.lineTo(e.sx,e.sy);ctx3.stroke();
    ctx3.fillStyle=a.c;ctx3.font='12px monospace';ctx3.fillText(a.l,e.sx+5,e.sy);
  });
  
  // Points sorted by depth
  let pp=pts.map(p=>({...proj(p.x,p.y,p.z),d:Math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)}));
  pp.sort((a,b)=>b.z-a.z);
  for(let p of pp){
    ctx3.fillStyle=distColor(p.d);
    ctx3.beginPath();ctx3.arc(p.sx,p.sy,Math.max(1,2*p.s*zoom),0,Math.PI*2);ctx3.fill();
  }
  
  // Rover position
  let rp=proj(roverX,roverY,0);
  ctx3.fillStyle='#f00';
  ctx3.beginPath();ctx3.arc(rp.sx,rp.sy,6,0,Math.PI*2);ctx3.fill();
}

// Mouse drag
c3d.onmousedown=e=>{dragging=true;mx=e.clientX;my=e.clientY;};
window.onmousemove=e=>{if(!dragging)return;rotY+=(e.clientX-mx)*0.01;rotX+=(e.clientY-my)*0.01;mx=e.clientX;my=e.clientY;};
window.onmouseup=()=>dragging=false;
c3d.onwheel=e=>{zoom*=e.deltaY>0?0.9:1.1;zoom=Math.max(0.05,Math.min(5,zoom));e.preventDefault();};

// WebSocket
const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=()=>{document.getElementById('st').textContent='OK';document.getElementById('st').className='stat-val ok';};
ws.onclose=()=>{document.getElementById('st').textContent='NO';document.getElementById('st').className='stat-val warn';};
ws.onmessage=e=>{
  let line=e.data;
  
  if(line.startsWith('W,')){
    let p=line.split(',');
    if(p.length>=4){
      let x=parseInt(p[1]), y=parseInt(p[2]), z=parseInt(p[3]);
      pts.push({x,y,z});
      if(pts.length>10000)pts.shift();
      document.getElementById('pts').textContent=pts.length;
    }
  }
  else if(line.startsWith('R,')){
    let p=line.split(',');
    if(p.length>=4){
      roverX=parseInt(p[1]);roverY=parseInt(p[2]);roverH=parseInt(p[3]);
      document.getElementById('rx').textContent=roverX;
      document.getElementById('ry').textContent=roverY;
      document.getElementById('rh').textContent=roverH;
      path.push([roverX,roverY]);
      if(path.length>1000)path.shift();
    }
  }
  else if(line.startsWith('D,')){
    let p=line.split(',');
    if(p.length>=2){
      document.getElementById('dist').textContent=p[1];
    }
  }
  else{
    log(line);
  }
};

function animate(){draw2D();draw3D();requestAnimationFrame(animate);}
animate();
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
    Serial.println("\nRover3D ESP32");

    pinMode(DANGER_PIN, OUTPUT);
    digitalWrite(DANGER_PIN, LOW);

    /* ToF init */
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(50);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(50);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (vl53.begin(0x29, &Wire)) {
        Serial.println("ToF OK");
        vl53.setTimingBudget(50);
        vl53.startRanging();
    } else {
        Serial.println("ToF FAIL");
    }

    /* WiFi AP */
    WiFi.softAP(ssid, password);
    Serial.print("AP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();
    ws.begin();
    ws.onEvent(wsEvent);

    /* UART from MSP432 */
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, -1);

    Serial.println("Ready");
}

/*===========================================================================*/
/* LOOP                                                                      */
/*===========================================================================*/

void loop() {
    static String buf;
    int pan, tilt, dist;
    int wx, wy, wz;

    server.handleClient();
    ws.loop();

    /* Continuously read ToF: drive DANGER_PIN for hazard avoidance
     * and keep the latest distance for the mapping scan */
    dist = 0;
    if (vl53.dataReady()) {
        dist = vl53.distance();
        vl53.clearInterrupt();
        vl53.startRanging();
        if (dist > 20 && dist < 4000) {
            last_dist = dist;
            digitalWrite(DANGER_PIN, (dist < CRITICAL_MM) ? HIGH : LOW);
            ws.broadcastTXT("D," + String(dist) + ",1");
        }
    }

    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n') {
            buf.trim();
            if (buf.length() > 0) {
                Serial.println("RX: " + buf);

                if (buf.startsWith("P,")) {
                    /* Servo position - use latest ToF reading to compute world point */
                    int c1 = buf.indexOf(',', 2);
                    if (c1 > 0) {
                        pan = buf.substring(2, c1).toInt();
                        tilt = buf.substring(c1 + 1).toInt();

                        /* Compute world coordinates from latest reading */
                        if (last_dist > 20 && last_dist < 4000) {
                            compute_world_point(pan, tilt, last_dist, &wx, &wy, &wz);

                            /* Send to dashboard */
                            ws.broadcastTXT("W," + String(wx) + "," + String(wy) + "," + String(wz));
                        }
                    }
                }
                else if (buf.startsWith("R,")) {
                    /* Rover position update */
                    int c1 = buf.indexOf(',', 2);
                    int c2 = buf.indexOf(',', c1 + 1);
                    if (c1 > 0 && c2 > 0) {
                        rover_x = buf.substring(2, c1).toFloat();
                        rover_y = buf.substring(c1 + 1, c2).toFloat();
                        rover_heading = buf.substring(c2 + 1).toFloat();

                        /* Forward to dashboard */
                        ws.broadcastTXT(buf);
                    }
                }
                else {
                    /* Forward other messages */
                    ws.broadcastTXT(buf);
                }
            }
            buf = "";
        } else if (c != '\r') {
            buf += c;
        }
    }
}