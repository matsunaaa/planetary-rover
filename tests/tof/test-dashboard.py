#!/usr/bin/env python3
"""
VL53L1X Real-time Dashboard
Run: python tof_dashboard.py
Then open: http://localhost:8080
"""

import serial
import serial.tools.list_ports
import threading
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import time

# Global data storage
data = {
    'distance': 0,
    'status': 0,
    'history': [],
    'max_history': 200
}
lock = threading.Lock()

HTML = '''<!DOCTYPE html>
<html>
<head>
    <title>VL53L1X Dashboard</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            background: #1a1a2e; 
            color: #eee; 
            margin: 0; 
            padding: 20px;
        }
        .container { max-width: 1200px; margin: 0 auto; }
        h1 { color: #0ff; text-align: center; }
        .stats {
            display: flex;
            justify-content: space-around;
            margin: 20px 0;
        }
        .stat-box {
            background: #16213e;
            padding: 30px 50px;
            border-radius: 10px;
            text-align: center;
        }
        .stat-value {
            font-size: 72px;
            font-weight: bold;
            color: #0ff;
        }
        .stat-label { color: #888; font-size: 18px; }
        .stat-unit { font-size: 24px; color: #888; }
        .status-ok { color: #0f0; }
        .status-err { color: #f00; }
        #chart {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            margin-top: 20px;
        }
        canvas { width: 100%; height: 300px; }
        .bar-container {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            margin-top: 20px;
        }
        .distance-bar {
            height: 60px;
            background: linear-gradient(90deg, #0ff, #00f);
            border-radius: 5px;
            transition: width 0.1s;
        }
        .bar-labels {
            display: flex;
            justify-content: space-between;
            margin-top: 10px;
            color: #888;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🛸 VL53L1X ToF Sensor Dashboard</h1>
        
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
        </div>
        
        <div class="bar-container">
            <div class="distance-bar" id="distBar" style="width: 0%"></div>
            <div class="bar-labels">
                <span>0mm</span>
                <span>500mm</span>
                <span>1000mm</span>
                <span>1500mm</span>
                <span>2000mm</span>
            </div>
        </div>
        
        <div id="chart">
            <canvas id="canvas"></canvas>
        </div>
    </div>
    
    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        let history = [];
        let lastUpdate = Date.now();
        let sampleCount = 0;
        
        function resize() {
            canvas.width = canvas.offsetWidth;
            canvas.height = 300;
        }
        resize();
        window.onresize = resize;
        
        function drawChart() {
            ctx.fillStyle = '#16213e';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            if (history.length < 2) return;
            
            // Grid
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;
            for (let y = 0; y <= 2000; y += 500) {
                let py = canvas.height - (y / 2000) * canvas.height;
                ctx.beginPath();
                ctx.moveTo(0, py);
                ctx.lineTo(canvas.width, py);
                ctx.stroke();
                ctx.fillStyle = '#666';
                ctx.fillText(y + 'mm', 5, py - 5);
            }
            
            // Line
            ctx.strokeStyle = '#0ff';
            ctx.lineWidth = 2;
            ctx.beginPath();
            for (let i = 0; i < history.length; i++) {
                let x = (i / (history.length - 1)) * canvas.width;
                let y = canvas.height - (history[i] / 2000) * canvas.height;
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
            
            // Fill
            ctx.lineTo(canvas.width, canvas.height);
            ctx.lineTo(0, canvas.height);
            ctx.closePath();
            ctx.fillStyle = 'rgba(0, 255, 255, 0.1)';
            ctx.fill();
        }
        
        async function update() {
            try {
                const res = await fetch('/data');
                const d = await res.json();
                
                document.getElementById('distance').textContent = d.distance;
                
                const statusEl = document.getElementById('status');
                // status 1 = OK, status 0 = WAIT
                const isOk = (d.status === 1);
                statusEl.textContent = isOk ? 'OK' : 'WAIT';
                statusEl.className = 'stat-value ' + (isOk ? 'status-ok' : 'status-err');
                
                // Update bar
                const pct = Math.min(100, (d.distance / 2000) * 100);
                document.getElementById('distBar').style.width = pct + '%';
                
                // Update history
                history = d.history;
                drawChart();
                
                // Sample rate
                sampleCount++;
                const now = Date.now();
                if (now - lastUpdate > 1000) {
                    document.getElementById('rate').textContent = sampleCount;
                    sampleCount = 0;
                    lastUpdate = now;
                }
            } catch (e) {}
            
            requestAnimationFrame(update);
        }
        
        update();
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
                self.wfile.write(json.dumps(data).encode())
        else:
            self.send_response(404)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass  # Suppress logs

def serial_reader(port):
    global data
    try:
        # Verified 115200 baud matching 12MHz SMCLK configuration
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Connected to {port} at 115200 baud")
        
        while True:
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            if raw_line.startswith('D,'):
                parts = raw_line.split(',')
                if len(parts) >= 3:
                    try:
                        dist = int(parts[1])
                        status = int(parts[2])
                        with lock:
                            data['distance'] = dist
                            data['status'] = status
                            data['history'].append(dist)
                            if len(data['history']) > data['max_history']:
                                data['history'].pop(0)
                    except ValueError:
                        pass
            elif raw_line:
                print(f"[MSP432] {raw_line}")
    except Exception as e:
        print(f"Serial error on {port}: {e}")

def find_port():
    ports = list(serial.tools.list_ports.comports())
    # Prioritize Application UART / User UART on MSP432 Launchpad
    for p in ports:
        desc = p.description.upper()
        if 'USER' in desc or 'APPLICATION' in desc or 'MSP' in desc:
            return p.device
    if ports:
        return ports[0].device
    return None

if __name__ == '__main__':
    port = find_port()
    if not port:
        print("No serial port found! Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        port = input("Enter port manually (e.g. COM5): ").strip()
    
    print(f"Using port: {port}")
    
    # Start serial reader thread
    t = threading.Thread(target=serial_reader, args=(port,), daemon=True)
    t.start()
    
    # Start web server
    print("Starting dashboard at http://localhost:8080")
    server = HTTPServer(('localhost', 8080), Handler)
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")