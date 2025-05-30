from flask import Flask, request, jsonify
from flask_cors import CORS
import socket
import threading
from collections import deque
from threading import Lock
import os
import logging

app = Flask(__name__)
CORS(app, resources={r"/api/*": {"origins": "*"}})

# TCP configuration (override with environment variables if needed)

logging.basicConfig(level=logging.DEBUG,
                    format="%(asctime)s [%(levelname)s] %(message)s")
app.logger.setLevel(logging.DEBUG)

# --- TCP configuration ---
TCP_IP = os.getenv("TCP_IP", "0.0.0.0")   # listen everywhere
TCP_PORT = int(os.getenv("TCP_PORT", 12000))

# Thread‑safe global state
messages = deque(maxlen=50)
msg_lock = Lock()

esp_socket = None
esp_lock = Lock()

last_sent_values = {}
PARAM_KEYS = [
    'Kp', 'Ki', 'Kd',
    'Kmp', 'Kmi', 'Kmd',
    'absolutemax_tilt', 'absolutemin_tilt',
    'motorspeed_setpoint'
]

# ---------- REST API ----------
@app.route("/api/send", methods=["POST"])
def send_command():
    data = request.get_json(force=True)
    command = data.get("command", "")
    
    app.logger.debug("HTTP /api/send received: %s", command)
    status = "Not sent (ESP not connected)"
    with esp_lock:
        if esp_socket:
            try:
                esp_socket.send(command.encode())
                status = "Command sent"
                app.logger.debug("→ forwarded to ESP: %s", command)
            except Exception as e:
                status = f"Failed to send: {e}"
    return jsonify({"status": status})

@app.route("/api/current_values")
def current_values():
    global last_sent_values
    force = request.args.get("force", "false").lower() == "true"
    with msg_lock:
        last_line = messages[-1] if messages else ""
    try:
        values = list(map(float, last_line.strip().split(",")))
        if len(values) == len(PARAM_KEYS):
            current = dict(zip(PARAM_KEYS, values))
            if force:
                last_sent_values = current.copy()
                return jsonify(current)
            delta = {
                k: v for k, v in current.items()
                if k not in last_sent_values or abs(last_sent_values[k] - v) > 1e-6
            }
            last_sent_values.update(delta)
            return jsonify(delta)
    except Exception:
        pass
    return jsonify({})

@app.route("/api/messages")
def get_messages():
    with msg_lock:
        return jsonify(list(messages))

# ---------- TCP server ----------
def tcp_server():
    global esp_socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((TCP_IP, TCP_PORT))
    server.listen(1)
    print(f"TCP Server listening on {TCP_IP}:{TCP_PORT}")
    while True:
        conn, addr = server.accept()
        print("ESP connected:", addr)
        with esp_lock:
            esp_socket = conn
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                with msg_lock:
                    messages.append(data.decode())
        finally:
            conn.close()

def start_tcp_thread():
    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()

if __name__ == "__main__":
    start_tcp_thread()
    app.run(host="0.0.0.0", port=8000, debug=False)

