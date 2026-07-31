"""
Python WebSocket Client Test

Connects to ESP32 WebSocket and prints received data.
Use this to test the WebSocket layer independently of the browser.

Usage:
  1. Connect PC to WiFi "ROVER3D" (password: 12345678)
  2. Run: python ws-client-test.py
  3. You should see live data printed

Requires: pip install websocket-client
"""

import websocket
import sys

def on_message(ws, message):
    print(f"RECV: {message}")

def on_error(ws, error):
    print(f"ERROR: {error}")

def on_open(ws):
    print("Connected to ESP32 WebSocket")

def on_close(ws, status, msg):
    print(f"Closed: {msg}")

if __name__ == "__main__":
    ws = websocket.WebSocketApp(
        "ws://192.168.4.1:81/",
        on_message=on_message,
        on_error=on_error,
        on_open=on_open,
        on_close=on_close
    )
    print("Connecting to ws://192.168.4.1:81/ ...")
    ws.run_forever()