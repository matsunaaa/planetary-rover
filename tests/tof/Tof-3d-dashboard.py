#!/usr/bin/env python3
"""
3D Scanner - Fixed Serial
"""

import serial
import serial.tools.list_ports
import numpy as np
import threading
import time
import struct
from http.server import HTTPServer, SimpleHTTPRequestHandler
import json
import os
import queue

# Config
SERIAL_BAUD = 115200
MAX_POINTS = 50000
SWEEP_RATE = 30.0  # deg/sec estimate

# Global state
points = []
raw_readings = []
lock = threading.Lock()
capturing = False
scan_start_time = 0
cmd_queue = queue.Queue()
serial_port = None

def polar_to_xyz(dist_mm, h_deg, v_deg):
    r = dist_mm
    h = np.radians(h_deg)
    v = np.radians(v_deg)
    x = r * np.cos(v) * np.sin(h)
    y = r * np.cos(v) * np.cos(h)
    z = r * np.sin(v)
    return x, y, z

def estimate_angles(ts_ms, start_ms):
    elapsed = (ts_ms - start_ms) / 1000.0
    h = (elapsed * SWEEP_RATE) % 360 - 180
    v = 0
    return h, v

def export_ply(filename, pts):
    with open(filename, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for x, y, z, t in pts:
            d = np.sqrt(x*x + y*y + z*z)
            ratio = min(1.0, d / 1300.0)
            r, g, b = int(255*ratio), 100, int(255*(1-ratio))
            f.write(f"{x:.2f} {y:.2f} {z:.2f} {r} {g} {b}\n")
    print(f"Exported {len(pts)} points to {filename}")

def export_obj(filename, pts):
    with open(filename, 'w') as f:
        f.write(f"# {len(pts)} points\n")
        for x, y, z, t in pts:
            f.write(f"v {x:.2f} {y:.2f} {z:.2f}\n")
    print(f"Exported {len(pts)} points to {filename}")

def export_stl(filename, pts):
    if len(pts) < 4:
        print("Need 4+ points")
        return False
    try:
        from scipy.spatial import Delaunay
        xyz = np.array([(p[0], p[1], p[2]) for p in pts])
        tri = Delaunay(xyz[:, :2])
        
        with open(filename, 'wb') as f:
            f.write(b'\x00' * 80)
            f.write(struct.pack('<I', len(tri.simplices)))
            for s in tri.simplices:
                v0, v1, v2 = xyz[s[0]], xyz[s[1]], xyz[s[2]]
                n = np.cross(v1-v0, v2-v0)
                norm = np.linalg.norm(n)
                if norm > 0: n /= norm
                f.write(struct.pack('<fff', *n))
                f.write(struct.pack('<fff', *v0))
                f.write(struct.pack('<fff', *v1))
                f.write(struct.pack('<fff', *v2))
                f.write(struct.pack('<H', 0))
        print(f"Exported {len(tri.simplices)} triangles to {filename}")
        return True
    except Exception as e:
        print(f"STL export failed: {e}")
        return False

def serial_thread(port_name):
    global points, raw_readings, capturing, scan_start_time, serial_port
    
    while True:
        try:
            print(f"Connecting to {port_name}...")
            ser = serial.Serial(port_name, SERIAL_BAUD, timeout=0.5)
            serial_port = ser
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.5)
            
            # Send ping
            ser.write(b'?')
            print("Sent ping, waiting for response...")
            
            while True:
                # Send any queued commands
                while not cmd_queue.empty():
                    try:
                        cmd = cmd_queue.get_nowait()
                        ser.write(cmd.encode())
                        print(f"Sent: {cmd}")
                        time.sleep(0.05)
                    except:
                        pass
                
                # Read line
                try:
                    line = ser.readline()
                    if not line:
                        continue
                    
                    line = line.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    print(f"[RX] {line}")
                    
                    if line.startswith('P,'):
                        parts = line.split(',')
                        if len(parts) >= 4:
                            try:
                                pid = int(parts[1])
                                dist = int(parts[2])
                                ts = int(parts[3])
                                
                                with lock:
                                    if scan_start_time == 0:
                                        scan_start_time = ts
                                    
                                    raw_readings.append((dist, ts))
                                    h, v = estimate_angles(ts, scan_start_time)
                                    x, y, z = polar_to_xyz(dist, h, v)
                                    points.append((x, y, z, ts))
                                    
                                    if len(points) > MAX_POINTS:
                                        points.pop(0)
                                        raw_readings.pop(0)
                            except ValueError as e:
                                print(f"Parse error: {e}")
                    
                    elif line == '!START':
                        with lock:
                            capturing = True
                            scan_start_time = 0
                        print(">>> CAPTURE STARTED")
                    
                    elif line == '!STOP':
                        with lock:
                            capturing = False
                        print(f">>> CAPTURE STOPPED ({len(points)} points)")
                    
                    elif line == '!CLEAR':
                        with lock:
                            points.clear()
                            raw_readings.clear()
                            scan_start_time = 0
                        print(">>> CLEARED")
                    
                    elif line == '!PING':
                        print(">>> PONG (connected)")
                        
                except serial.SerialException as e:
                    print(f"Serial read error: {e}")
                    break
                    
        except Exception as e:
            print(f"Serial error: {e}")
            serial_port = None
            time.sleep(2)

HTML = '''<!DOCTYPE html>
<html>
<head>
    <title>3D Scanner</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: Arial; background: #0a0a0f; color: #fff; overflow: hidden; }
        .container { display: flex; height: 100vh; }
        .sidebar { width: 280px; background: #111; padding: 15px; overflow-y: auto; }
        .viewer { flex: 1; }
        h1 { font-size: 16px; color: #0ff; margin-bottom: 15px; }
        .stat { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #222; }
        .stat-value { color: #0ff; }
        .btn { width: 100%; padding: 12px; margin: 4px 0; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; }
        .btn-start { background: #0a0; color: #fff; }
        .btn-stop { background: #a00; color: #fff; }
        .btn-export { background: #06a; color: #fff; }
        .btn-clear { background: #333; color: #fff; }
        .btn:hover { opacity: 0.8; }
        .status { padding: 10px; margin: 10px 0; border-radius: 4px; text-align: center; }
        .status-on { background: #0a02; border: 1px solid #0a0; color: #0f0; }
        .status-off { background: #3332; border: 1px solid #333; }
        canvas { width: 100%; height: 100%; }
        .log { margin-top: 10px; padding: 8px; background: #000; font-family: monospace; font-size: 11px; height: 100px; overflow-y: auto; }
    </style>
</head>
<body>
    <div class="container">
        <div class="sidebar">
            <h1>3D Scanner</h1>
            <div id="status" class="status status-off">IDLE</div>
            <div class="stat"><span>Points</span><span class="stat-value" id="pts">0</span></div>
            <div class="stat"><span>Last</span><span class="stat-value" id="last">--</span></div>
            
            <button class="btn btn-start" onclick="send('s')">START</button>
            <button class="btn btn-stop" onclick="send('x')">STOP</button>
            <button class="btn btn-clear" onclick="send('c')">CLEAR</button>
            <hr style="margin:10px 0;border-color:#333">
            <button class="btn btn-export" onclick="exp('ply')">Export PLY</button>
            <button class="btn btn-export" onclick="exp('stl')">Export STL</button>
            <button class="btn btn-export" onclick="exp('obj')">Export OBJ</button>
            
            <div class="log" id="log"></div>
        </div>
        <div class="viewer"><canvas id="c"></canvas></div>
    </div>
<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let pts = [], rot = {x:-30, y:45}, zoom = 0.3, drag = false, last = {x:0,y:0};

function resize() { canvas.width = canvas.offsetWidth; canvas.height = canvas.offsetHeight; }
resize(); window.onresize = resize;

canvas.onmousedown = e => { drag = true; last = {x:e.clientX, y:e.clientY}; };
canvas.onmouseup = () => drag = false;
canvas.onmousemove = e => { if(drag) { rot.y += (e.clientX-last.x)*0.5; rot.x += (e.clientY-last.y)*0.5; last={x:e.clientX,y:e.clientY}; }};
canvas.onwheel = e => { zoom *= e.deltaY > 0 ? 0.9 : 1.1; zoom = Math.max(0.05, Math.min(2, zoom)); e.preventDefault(); };

function proj(x,y,z) {
    const rx = rot.x * Math.PI/180, ry = rot.y * Math.PI/180;
    const y1 = y*Math.cos(rx) - z*Math.sin(rx), z1 = y*Math.sin(rx) + z*Math.cos(rx);
    const x2 = x*Math.cos(ry) + z1*Math.sin(ry), z2 = -x*Math.sin(ry) + z1*Math.cos(ry);
    return { x: canvas.width/2 + x2*zoom, y: canvas.height/2 - y1*zoom, z: z2 };
}

function draw() {
    ctx.fillStyle = '#0a0a0f'; ctx.fillRect(0,0,canvas.width,canvas.height);
    
    // Grid
    ctx.strokeStyle = '#222'; ctx.lineWidth = 1;
    for(let i=-500;i<=500;i+=100) {
        let p1=proj(i,0,-500), p2=proj(i,0,500); ctx.beginPath(); ctx.moveTo(p1.x,p1.y); ctx.lineTo(p2.x,p2.y); ctx.stroke();
        p1=proj(-500,0,i); p2=proj(500,0,i); ctx.beginPath(); ctx.moveTo(p1.x,p1.y); ctx.lineTo(p2.x,p2.y); ctx.stroke();
    }
    
    // Axes
    let o=proj(0,0,0), ax=proj(150,0,0), ay=proj(0,150,0), az=proj(0,0,150);
    ctx.lineWidth=2;
    ctx.strokeStyle='#f00'; ctx.beginPath(); ctx.moveTo(o.x,o.y); ctx.lineTo(ax.x,ax.y); ctx.stroke();
    ctx.strokeStyle='#0f0'; ctx.beginPath(); ctx.moveTo(o.x,o.y); ctx.lineTo(ay.x,ay.y); ctx.stroke();
    ctx.strokeStyle='#00f'; ctx.beginPath(); ctx.moveTo(o.x,o.y); ctx.lineTo(az.x,az.y); ctx.stroke();
    
    // Points
    const projected = pts.map(p => ({...proj(p.x, p.z, p.y), d: Math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)}));
    projected.sort((a,b) => a.z - b.z);
    for(const p of projected) {
        const ratio = Math.min(1, p.d/1300);
        ctx.fillStyle = `rgb(${Math.floor(255*ratio)},100,${Math.floor(255*(1-ratio))})`;
        ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, Math.PI*2); ctx.fill();
    }
    
    ctx.fillStyle='#fff'; ctx.font='12px monospace';
    ctx.fillText(`${pts.length} points - drag rotate, scroll zoom`, 10, 20);
    requestAnimationFrame(draw);
}
draw();

function log(msg) {
    const el = document.getElementById('log');
    el.innerHTML += msg + '<br>';
    el.scrollTop = el.scrollHeight;
}

async function poll() {
    try {
        const r = await fetch('/data');
        const d = await r.json();
        pts = d.points.map(p => ({x:p[0], y:p[1], z:p[2]}));
        document.getElementById('pts').textContent = d.count;
        document.getElementById('last').textContent = d.last ? d.last + ' mm' : '--';
        document.getElementById('status').textContent = d.capturing ? 'CAPTURING' : 'IDLE';
        document.getElementById('status').className = 'status ' + (d.capturing ? 'status-on' : 'status-off');
    } catch(e) {}
    setTimeout(poll, 200);
}
poll();

async function send(cmd) {
    log('> ' + cmd);
    await fetch('/cmd?c=' + cmd);
}

function exp(fmt) { window.location = '/export/' + fmt; }
</script>
</body>
</html>
'''

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
            with lock:
                d = {
                    'count': len(points),
                    'capturing': capturing,
                    'points': [(p[0], p[1], p[2]) for p in points[-3000:]],
                    'last': raw_readings[-1][0] if raw_readings else None
                }
            self.wfile.write(json.dumps(d).encode())
            
        elif self.path.startswith('/cmd'):
            cmd = self.path.split('=')[1] if '=' in self.path else ''
            if cmd:
                cmd_queue.put(cmd)
            self.send_response(200)
            self.end_headers()
            
        elif self.path == '/export/ply':
            with lock: pts = list(points)
            fn = f'scan_{int(time.time())}.ply'
            export_ply(fn, pts)
            self._send_file(fn)
            
        elif self.path == '/export/obj':
            with lock: pts = list(points)
            fn = f'scan_{int(time.time())}.obj'
            export_obj(fn, pts)
            self._send_file(fn)
            
        elif self.path == '/export/stl':
            with lock: pts = list(points)
            fn = f'scan_{int(time.time())}.stl'
            if export_stl(fn, pts):
                self._send_file(fn)
            else:
                self.send_response(500)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()
    
    def _send_file(self, fn):
        if os.path.exists(fn):
            self.send_response(200)
            self.send_header('Content-type', 'application/octet-stream')
            self.send_header('Content-Disposition', f'attachment; filename="{fn}"')
            self.end_headers()
            with open(fn, 'rb') as f:
                self.wfile.write(f.read())
            os.remove(fn)
    
    def log_message(self, *args): pass

def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.upper()
        if 'USER' in desc or 'APPLICATION' in desc or 'MSP' in desc:
            return p.device
    if ports:
        return ports[0].device
    return None

if __name__ == '__main__':
    print("="*40)
    print("  3D Point Cloud Scanner")
    print("="*40)
    
    port = find_port()
    if not port:
        print("\nPorts available:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        port = input("Enter port: ").strip()
    
    print(f"Port: {port}")
    
    t = threading.Thread(target=serial_thread, args=(port,), daemon=True)
    t.start()
    
    print("\nDashboard: http://localhost:8080\n")
    
    server = HTTPServer(('0.0.0.0', 8080), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nBye")