import serial
import threading
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import time

# ============== CONFIG ==============
BT_PORT = 'COM13'
BAUD = 115200
HTTP_PORT = 8080

# ============== GLOBALS ==============
points = []
points_lock = threading.Lock()
connected = False
last_msg = ""

# ============== SERIAL READER ==============
def serial_thread():
    global connected, last_msg
    
    while True:
        try:
            ser = serial.Serial(BT_PORT, BAUD, timeout=1)
            connected = True
            print(f"Connected to {BT_PORT}")
            
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                last_msg = line
                
                if line.startswith('P,'):
                    parts = line.split(',')
                    if len(parts) >= 4:
                        try:
                            x = int(parts[1])
                            y = int(parts[2])
                            z = int(parts[3])
                            with points_lock:
                                points.append({'x': x, 'y': y, 'z': z})
                            print(f"Point: ({x}, {y}, {z}) - Total: {len(points)}")
                        except ValueError:
                            print(f"Parse error: {line}")
                else:
                    print(f"Unknown: {line}")
                    
        except serial.SerialException as e:
            connected = False
            print(f"Serial error: {e}")
            print("Retrying in 2s...")
            time.sleep(2)

# ============== HTTP HANDLER ==============
class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        
        elif self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            with points_lock:
                data = {
                    'connected': connected,
                    'last_msg': last_msg,
                    'count': len(points),
                    'points': points[-500:]
                }
            self.wfile.write(json.dumps(data).encode('utf-8'))
        
        elif self.path == '/clear':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            with points_lock:
                points.clear()
            self.wfile.write(b'OK')
        
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass

# ============== HTML PAGE ==============
HTML_PAGE = '''<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Rover 3D Dashboard</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: monospace; background: #1a1a2e; color: #eee; padding: 20px; }
h1 { color: #0ff; margin-bottom: 10px; }
.status { margin-bottom: 20px; padding: 10px; background: #16213e; border-radius: 5px; }
.connected { color: #0f0; }
.disconnected { color: #f00; }
.container { display: flex; gap: 20px; }
.panel { background: #16213e; padding: 15px; border-radius: 5px; }
#plot2d { background: #0d1117; border: 1px solid #333; }
#plot3d { background: #0d1117; border: 1px solid #333; }
button { background: #0ff; color: #000; border: none; padding: 8px 16px; margin: 5px; cursor: pointer; font-family: monospace; }
button:hover { background: #0aa; }
.info { margin-top: 10px; font-size: 12px; color: #888; }
</style>
</head>
<body>
<h1>Rover 3D Mapping Dashboard</h1>

<div class="status">
    <span id="connStatus" class="disconnected">DISCONNECTED</span> | 
    Points: <span id="pointCount">0</span> | 
    Last: <span id="lastMsg">-</span>
</div>

<div>
    <button onclick="clearPoints()">Clear Points</button>
    <button onclick="toggleRotate()">Toggle Rotate</button>
</div>

<div class="container">
    <div class="panel">
        <h3>Top-Down View (X-Y)</h3>
        <canvas id="plot2d" width="400" height="400"></canvas>
        <div class="info">Grid: 100 units</div>
    </div>
    <div class="panel">
        <h3>3D View</h3>
        <canvas id="plot3d" width="500" height="400"></canvas>
        <div class="info">Drag to rotate</div>
    </div>
</div>

<script>
let points = [];
let rotateAngle = 0;
let autoRotate = true;
let dragStart = null;
let angleX = 0.5;
let angleY = 0.3;

const canvas2d = document.getElementById('plot2d');
const ctx2d = canvas2d.getContext('2d');
const canvas3d = document.getElementById('plot3d');
const ctx3d = canvas3d.getContext('2d');

// Mouse drag for 3D view
canvas3d.addEventListener('mousedown', e => {
    dragStart = {x: e.offsetX, y: e.offsetY, ax: angleX, ay: angleY};
    autoRotate = false;
});
canvas3d.addEventListener('mousemove', e => {
    if (dragStart) {
        angleY = dragStart.ay + (e.offsetX - dragStart.x) * 0.01;
        angleX = dragStart.ax + (e.offsetY - dragStart.y) * 0.01;
    }
});
canvas3d.addEventListener('mouseup', () => { dragStart = null; });
canvas3d.addEventListener('mouseleave', () => { dragStart = null; });

function toggleRotate() {
    autoRotate = !autoRotate;
}

function clearPoints() {
    fetch('/clear').then(() => { points = []; });
}

function draw2D() {
    const w = canvas2d.width;
    const h = canvas2d.height;
    const cx = w / 2;
    const cy = h / 2;
    const scale = 2;
    
    ctx2d.fillStyle = '#0d1117';
    ctx2d.fillRect(0, 0, w, h);
    
    // Grid
    ctx2d.strokeStyle = '#222';
    ctx2d.lineWidth = 1;
    for (let i = -200; i <= 200; i += 50) {
        ctx2d.beginPath();
        ctx2d.moveTo(cx + i * scale, 0);
        ctx2d.lineTo(cx + i * scale, h);
        ctx2d.stroke();
        ctx2d.beginPath();
        ctx2d.moveTo(0, cy - i * scale);
        ctx2d.lineTo(w, cy - i * scale);
        ctx2d.stroke();
    }
    
    // Axes
    ctx2d.strokeStyle = '#444';
    ctx2d.lineWidth = 2;
    ctx2d.beginPath();
    ctx2d.moveTo(cx, 0);
    ctx2d.lineTo(cx, h);
    ctx2d.moveTo(0, cy);
    ctx2d.lineTo(w, cy);
    ctx2d.stroke();
    
    // Origin marker (rover position)
    ctx2d.fillStyle = '#f00';
    ctx2d.beginPath();
    ctx2d.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx2d.fill();
    
    // Points
    ctx2d.fillStyle = '#0ff';
    points.forEach(p => {
        const px = cx + p.x * scale;
        const py = cy - p.y * scale;
        ctx2d.beginPath();
        ctx2d.arc(px, py, 2, 0, Math.PI * 2);
        ctx2d.fill();
    });
}

function draw3D() {
    const w = canvas3d.width;
    const h = canvas3d.height;
    const cx = w / 2;
    const cy = h / 2;
    const scale = 1.5;
    
    ctx3d.fillStyle = '#0d1117';
    ctx3d.fillRect(0, 0, w, h);
    
    if (autoRotate) {
        angleY += 0.01;
    }
    
    const cosX = Math.cos(angleX);
    const sinX = Math.sin(angleX);
    const cosY = Math.cos(angleY);
    const sinY = Math.sin(angleY);
    
    function project(x, y, z) {
        let px = x * cosY - z * sinY;
        let pz = x * sinY + z * cosY;
        let py = y * cosX - pz * sinX;
        pz = y * sinX + pz * cosX;
        const depth = 300;
        const factor = depth / (depth + pz);
        return {
            x: cx + px * scale * factor,
            y: cy - py * scale * factor,
            z: pz
        };
    }
    
    // Draw axes
    const axisLen = 80;
    const axes = [
        {p1: [0,0,0], p2: [axisLen,0,0], c: '#f00', l: 'X'},
        {p1: [0,0,0], p2: [0,axisLen,0], c: '#0f0', l: 'Y'},
        {p1: [0,0,0], p2: [0,0,axisLen], c: '#00f', l: 'Z'}
    ];
    axes.forEach(a => {
        const p1 = project(a.p1[0], a.p1[1], a.p1[2]);
        const p2 = project(a.p2[0], a.p2[1], a.p2[2]);
        ctx3d.strokeStyle = a.c;
        ctx3d.lineWidth = 2;
        ctx3d.beginPath();
        ctx3d.moveTo(p1.x, p1.y);
        ctx3d.lineTo(p2.x, p2.y);
        ctx3d.stroke();
        ctx3d.fillStyle = a.c;
        ctx3d.fillText(a.l, p2.x + 5, p2.y);
    });
    
    // Sort points by depth for proper rendering
    const projected = points.map(p => {
        const proj = project(p.x, p.y, p.z);
        return {...proj, orig: p};
    }).sort((a, b) => b.z - a.z);
    
    // Draw points
    projected.forEach(p => {
        const brightness = Math.max(50, 255 - p.z * 0.5);
        ctx3d.fillStyle = `rgb(0, ${brightness}, ${brightness})`;
        const size = Math.max(1, 3 - p.z * 0.01);
        ctx3d.beginPath();
        ctx3d.arc(p.x, p.y, size, 0, Math.PI * 2);
        ctx3d.fill();
    });
}

function updateStatus(data) {
    document.getElementById('connStatus').textContent = data.connected ? 'CONNECTED' : 'DISCONNECTED';
    document.getElementById('connStatus').className = data.connected ? 'connected' : 'disconnected';
    document.getElementById('pointCount').textContent = data.count;
    document.getElementById('lastMsg').textContent = data.last_msg || '-';
}

function fetchData() {
    fetch('/data')
        .then(r => r.json())
        .then(data => {
            points = data.points || [];
            updateStatus(data);
        })
        .catch(e => {
            document.getElementById('connStatus').textContent = 'SERVER ERROR';
            document.getElementById('connStatus').className = 'disconnected';
        });
}

function animate() {
    draw2D();
    draw3D();
    requestAnimationFrame(animate);
}

setInterval(fetchData, 200);
animate();
</script>
</body>
</html>
'''

# ============== MAIN ==============
if __name__ == '__main__':
    print("=" * 40)
    print("Rover 3D Dashboard")
    print("=" * 40)
    print(f"Bluetooth: {BT_PORT} @ {BAUD}")
    print(f"Dashboard: http://localhost:{HTTP_PORT}")
    print("=" * 40)
    
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    
    server = HTTPServer(('', HTTP_PORT), Handler)
    print("Server running... Open browser to dashboard URL")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutdown")