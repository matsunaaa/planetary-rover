import serial
import serial.tools.list_ports
import threading
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import time

DATA = {
    'connected': False,
    'distance': 0,
    'status': 0,
    'history': [],
    'max_history': 200,
    'last_msg': '',
    'points': [],
    'count': 0
}
LOCK = threading.Lock()

def serial_reader(port, baud=115200):
    global DATA
    try:
        ser = serial.Serial(port, baud, timeout=1)
        with LOCK:
            DATA['connected'] = True
        print(f"Connected to {port} at {baud} baud")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            with LOCK:
                DATA['last_msg'] = line
            if line.startswith('D,'):
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        dist = int(parts[1])
                        st = int(parts[2])
                        with LOCK:
                            DATA['distance'] = dist
                            DATA['status'] = st
                            DATA['history'].append(dist)
                            if len(DATA['history']) > DATA['max_history']:
                                DATA['history'].pop(0)
                    except ValueError:
                        pass
            elif line.startswith('P,'):
                parts = line.split(',')
                if len(parts) >= 4:
                    try:
                        x = int(parts[1])
                        y = int(parts[2])
                        z = int(parts[3])
                        with LOCK:
                            DATA['points'].append({'x': x, 'y': y, 'z': z})
                            DATA['count'] = len(DATA['points'])
                        print(f"Point: ({x}, {y}, {z}) - Total: {len(DATA['points'])}")
                    except ValueError:
                        pass
            elif line.startswith('S:') or line.startswith('T,'):
                print(f"[MSP432] {line}")
    except serial.SerialException as e:
        with LOCK:
            DATA['connected'] = False
        print(f"Serial error on {port}: {e}")

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.upper()
        if 'USER' in desc or 'APPLICATION' in desc or 'MSP' in desc:
            return p.device
    for p in ports:
        desc = p.description.upper()
        if 'BLUETOOTH' in desc or 'ESP32' in desc or 'SPP' in desc or 'ROVER3D' in desc:
            return p.device
    if ports:
        return ports[0].device
    return None

def build_html():
    return '''<!DOCTYPE html>
<html>
<head>
<title>Rover ToF Dashboard</title>
<style>
body { background: #1a1a2e; color: #eee; font-family: Arial, sans-serif; margin: 0; padding: 20px; }
.container { max-width: 1200px; margin: 0 auto; }
h1 { color: #0ff; text-align: center; }
.stats { display: flex; justify-content: space-around; margin: 20px 0; flex-wrap: wrap; }
.stat-box { background: #16213e; padding: 20px 40px; border-radius: 10px; text-align: center; margin: 10px; }
.stat-value { font-size: 48px; font-weight: bold; color: #0ff; }
.stat-label { color: #888; font-size: 18px; }
.stat-unit { font-size: 24px; color: #888; }
.status-ok { color: #0f0; }
.status-err { color: #f00; }
#chart { background: #16213e; border-radius: 10px; padding: 20px; margin-top: 20px; }
canvas { width: 100%; height: 300px; }
.bar-container { background: #16213e; border-radius: 10px; padding: 20px; margin-top: 20px; }
.distance-bar { height: 60px; background: linear-gradient(90deg, #0ff, #00f); border-radius: 5px; transition: width 0.1s; }
.bar-labels { display: flex; justify-content: space-between; margin-top: 10px; color: #888; }
.panel { display: flex; gap: 20px; margin-top: 20px; }
.panel > div { flex: 1; background: #16213e; border-radius: 10px; padding: 20px; }
.status-text { margin-top: 20px; color: #888; font-size: 14px; word-break: break-all; }
button { background: #0ff; color: #000; border: none; padding: 5px 15px; margin: 5px; cursor: pointer; border-radius: 3px; }
</style>
</head>
<body>
<div class="container">
    <h1>Rover ToF Dashboard</h1>
    <div class="stats">
        <div class="stat-box">
            <div class="stat-value" id="distance">--</div>
            <div class="stat-label">Distance <span class="stat-unit">mm</span></div>
        </div>
        <div class="stat-box">
            <div class="stat-value" id="status">--</div>
            <div class="stat-label">Status</div>
        </div>
        <div class="stat-box">
            <div class="stat-value" id="rate">--</div>
            <div class="stat-label">Samples/sec</div>
        </div>
        <div class="stat-box">
            <div class="stat-value" id="conn">--</div>
            <div class="stat-label">Serial</div>
        </div>
    </div>
    <div class="bar-container">
        <div class="distance-bar" id="distBar" style="width: 0%"></div>
        <div class="bar-labels">
            <span>0mm</span><span>500mm</span><span>1000mm</span><span>1500mm</span><span>2000mm</span>
        </div>
    </div>
    <div class="panel">
        <div>
            <h3>3D Point Cloud</h3>
            <canvas id="c2d" width="400" height="400" style="width:100%;height:auto;background:#000;border:1px solid #333;"></canvas>
            <div style="margin-top:10px">
                <button onclick="fetch('/clear')">Clear Points</button>
                <span style="color:#888">Points: <span id="ptCnt">0</span></span>
            </div>
        </div>
        <div>
            <h3>Distance History</h3>
            <canvas id="canvas"></canvas>
        </div>
    </div>
    <div class="status-text">
        Last message: <span id="lastMsg">-</span>
    </div>
</div>
<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const c2d = document.getElementById('c2d').getContext('2d');
let history = [];
let pts = [];
let lastUpdate = Date.now();
let sampleCount = 0;

function resize() { canvas.width = canvas.offsetWidth; canvas.height = 300; }
resize();
window.onresize = resize;

function drawChart() {
    ctx.fillStyle = '#16213e';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (history.length < 2) return;
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    for (let y = 0; y <= 2000; y += 500) {
        let py = canvas.height - (y / 2000) * canvas.height;
        ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(canvas.width, py); ctx.stroke();
        ctx.fillStyle = '#666'; ctx.fillText(y + 'mm', 5, py - 5);
    }
    ctx.strokeStyle = '#0ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < history.length; i++) {
        let x = (i / (history.length - 1)) * canvas.width;
        let y = canvas.height - (history[i] / 2000) * canvas.height;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.lineTo(canvas.width, canvas.height);
    ctx.lineTo(0, canvas.height);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0, 255, 255, 0.1)';
    ctx.fill();
}

function draw2D() {
    c2d.fillStyle = '#000';
    c2d.fillRect(0, 0, 400, 400);
    if (pts.length === 0) return;
    let maxVal = 1;
    pts.forEach(p => { maxVal = Math.max(maxVal, Math.abs(p.x), Math.abs(p.y)); });
    const scale = 150 / maxVal;
    c2d.strokeStyle = '#222';
    c2d.beginPath();
    for (let i = 0; i <= 400; i += 50) { c2d.moveTo(i, 0); c2d.lineTo(i, 400); c2d.moveTo(0, i); c2d.lineTo(400, i); }
    c2d.stroke();
    c2d.fillStyle = '#0ff';
    pts.forEach(p => {
        let px = 200 + p.x * scale, py = 200 - p.y * scale;
        if (px >= 0 && px <= 400 && py >= 0 && py <= 400) { c2d.beginPath(); c2d.arc(px, py, 3, 0, 6.28); c2d.fill(); }
    });
}

async function update() {
    try {
        const r = await fetch('/data');
        const d = await r.json();
        document.getElementById('distance').textContent = d.distance;
        const se = document.getElementById('status');
        const isOk = (d.status === 1);
        se.textContent = isOk ? 'OK' : 'WAIT';
        se.className = 'stat-value ' + (isOk ? 'status-ok' : 'status-err');
        const pct = Math.min(100, (d.distance / 2000) * 100);
        document.getElementById('distBar').style.width = pct + '%';
        document.getElementById('conn').textContent = d.connected ? 'OK' : 'NO';
        document.getElementById('conn').style.color = d.connected ? '#0f0' : '#f00';
        document.getElementById('lastMsg').textContent = d.last_msg || '-';
        history = d.history || [];
        pts = d.points || [];
        document.getElementById('ptCnt').textContent = d.count || 0;
        drawChart();
        draw2D();
        sampleCount++;
        const now = Date.now();
        if (now - lastUpdate > 1000) {
            document.getElementById('rate').textContent = sampleCount;
            sampleCount = 0;
            lastUpdate = now;
        }
    } catch(e) {}
    requestAnimationFrame(update);
}
update();
</script>
</body>
</html>'''

class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with LOCK:
                payload = {
                    'connected': DATA['connected'],
                    'distance': DATA['distance'],
                    'status': DATA['status'],
                    'history': DATA['history'][-200:],
                    'last_msg': DATA['last_msg'],
                    'points': DATA['points'][-500:],
                    'count': DATA['count']
                }
            self.wfile.write(json.dumps(payload).encode())
        elif self.path == '/clear':
            self.send_response(200)
            self.end_headers()
            with LOCK:
                DATA['points'].clear()
                DATA['count'] = 0
        else:
            self.send_response(404)
            self.end_headers()
    def log_message(self, *args):
        pass

HTML = build_html()

if __name__ == '__main__':
    port = find_port()
    if not port:
        print("No serial port found! Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        port = input("Enter port manually (e.g. COM6): ").strip()
    print(f"Using port: {port}")
    t = threading.Thread(target=serial_reader, args=(port,), daemon=True)
    t.start()
    print("Starting dashboard at http://localhost:8080")
    server = HTTPServer(('localhost', 8080), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")