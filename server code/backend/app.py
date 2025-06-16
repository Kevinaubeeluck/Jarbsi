from flask import Flask, request, jsonify
from flask_cors import CORS
from create_db import (
    get_battery, set_battery,
    add_telemetry, latest_telemetry   # ← NEW
)
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
messages = deque(maxlen=300)
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

def send_to_esp(payload: str):
    """Thread-safe write to the current ESP socket (if any)."""
    with esp_lock:
        sock = esp_socket          
    if sock:                       
        try:
            sock.send(payload.encode())
        except OSError:
            pass


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


@app.route("/api/telemetry")
def api_telemetry():
    row = latest_telemetry()
    if not row:
        return jsonify({})
    return jsonify({
        "level"   : row.bat_pct,
        "bat_volt": row.bat_volt,
        "v5_volt" : row.v5_volt,
        "mot_volt": row.mot_volt,
        "mot_curr": row.mot_curr,
        "v5_curr" : row.v5_curr,
        "ts"      : row.created_at.isoformat()
    })


leaderboard_data = [
    {"name": "StarPlatinum", "score": 1.00},
    {"name": "TheWorld", "score": 1.01},
    {"name": "GER", "score": 2.00}
]

@app.route("/api/batteryinfo", methods=["GET", "POST"])
def battery_info():
    if request.method == "POST":
        level = float(request.get_json(force=True)["level"])
        set_battery(level)                         
        send_to_esp(f"BAT:{level:.2f}\n")          
        return jsonify({"status": "stored", "level": level})

    level = get_battery()
    return jsonify({"level": level if level is not None else "unknown"})




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
            bat = get_battery()
            if bat is not None:
                msg = f"BAT:{bat:.2f}\n"
                conn.send(msg.encode())
                app.logger.debug("→ sent stored battery %s to ESP", msg.strip())
            partial = {}                             # collect the six lines

            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break

                for raw in chunk.decode().splitlines():
                    with msg_lock:
                        messages.append(raw)

                    if raw.startswith("BAT:"):
                        partial["bat_pct"] = float(raw.split(":",1)[1])
                    elif raw.startswith("BATVOLT:"):
                        partial["bat_volt"] = float(raw.split(":",1)[1])
                    elif raw.startswith("5VVOLT:"):
                        partial["v5_volt"]  = float(raw.split(":",1)[1])
                    elif raw.startswith("MOTVOLT:"):
                        partial["mot_volt"] = float(raw.split(":",1)[1])
                    elif raw.startswith("MOTCURR:"):
                        partial["mot_curr"] = float(raw.split(":",1)[1])
                    elif raw.startswith("5VCURR:"):
                        partial["v5_curr"]  = float(raw.split(":",1)[1])

                        # ← last line of the block; store and reset
                        add_telemetry(**partial)
                        partial.clear()

        finally:
            conn.close()
            with esp_lock:
                esp_socket = None

def start_tcp_thread():
    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()

if __name__ == "__main__":
    start_tcp_thread()
    app.run(host="0.0.0.0", port=8000, debug=False)