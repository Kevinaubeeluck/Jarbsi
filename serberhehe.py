from flask import Flask, request, render_template_string, jsonify
import socket
import threading

from collections import deque
from threading import Lock

# Thread-safe storage for received messages
messages = deque(maxlen=50)  # Store last 50 messages
msg_lock = Lock()

esp_socket = None  # Global variable to hold active ESP connection
esp_lock = Lock()


# TCP Server details
server_ip = '192.168.219.234'
server_port = 12000


app = Flask(__name__)

html_page = """
<!DOCTYPE html>
<html>
<head>
    <title>Device Controller</title>
    <style>
        body {
            background-color: #f2f2f2;
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            padding: 20px;
            gap: 40px; 
            flex-wrap: wrap; 
        }
        .container {
            text-align: center;
            background-color: white;
            padding: 40px;
            border-radius: 15px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
            max-width: 800px;  
            width: 100%;
            
        }

        #messageDisplay {
            text-align: left;
            max-height: 200px;
            overflow: auto;
            border: 1px solid #ccc;
            padding: 10px;
            border-radius: 8px;
            word-break: break-word;
            white-space: pre-wrap;
            max-width: 90%;
            margin: 0 auto;
        }


        h1 {
            font-size: 2em;
            color: #333;
        }
        input[type="text"] {
            padding: 10px;
            font-size: 1em;
            width: 60%;
            border: 1px solid #ccc;
            border-radius: 8px;
            margin-top: 20px;
        }
        input[type="submit"] {
            margin-top: 20px;
            padding: 10px 20px;
            font-size: 1em;
            background-color: #007bff;
            color: white;
            border: none;
            border-radius: 8px;
            cursor: pointer;
        }
        input[type="submit"]:hover {
            background-color: #0056b3;
        }
        .grid-container {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 20px;
            margin-top: 30px;
            max-width: 1000px;
            
        }

        .param-box {
            background-color: #f9f9f9;
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            text-align: center;
            max-width: 250px;
            margin: 0 auto;
        }

        .param-box h3 {
            margin-bottom: 10px;
            font-size: 1em;
        }


        .param-box input {
            width: 100%;
            padding: 8px;
            margin-bottom: 10px;
            border-radius: 6px;
            border: 1px solid #ccc;
        }

        .param-box button {
            padding: 8px 12px;
            background-color: #007bff;
            color: white;
            border: none;
            border-radius: 6px;
            cursor: pointer;
        }

        .param-box button:hover {
            background-color: #0056b3;
        }

    </style>
</head>


<div class="grid-container">
    {% for param in ['Kp', 'Ki', 'Kd', 'Kmp', 'Kmi', 'Kmd', 'absolutemax_tilt', 'absolutemin_tilt', 'motorspeed_setpoint'] %}
    <div class="param-box">
    <h3>{{ param }}</h3>
        <form onsubmit="sendParam(event, '{{ param }}')">
        <input type="text" id="{{ param }}Input" placeholder="Enter {{ param | e}}" required>
        <button type="submit">Send</button>
    </form>
    </div>  
    {% endfor %}
</div>

<div style="width: 100%; display: flex; justify-content: center;">
    <div class="container">
        <h2>Messages from ESP:</h2>
        <div id="messageDisplay"></div>
    </div>
</div>





<script>
    async function fetchMessages() {
        try {
            const response = await fetch('/messages');
            const data = await response.json();
            const display = document.getElementById('messageDisplay');
            display.innerHTML = data.map(msg => `<div>${msg}</div>`).join('');
        } catch (e) {
            console.error("Failed to fetch messages:", e);
        }
    }
    async function sendPing() {
        try {
            const response = await fetch('/ping', {
                method: 'POST'
            });
            const result = await response.json();
            console.log("Ping response:", result);
            fetchMessages(); // Refresh after ping
        } catch (e) {
            console.error("Ping failed:", e);
        }
    }
        async function sendParam(event, paramName) {
        event.preventDefault(); 
        const value = document.getElementById(`${paramName}Input`).value;

        let command;
        switch (paramName) {
            case 'Kp':
                command = `SET_KP(${value})`;
                break;
            case 'Ki':
                command = `SET_KI(${value})`;
                break;
            case 'Kd':
                command = `SET_KD(${value})`;
                break;
            case 'Kmp':
                command = `SET_KMP(${value})`;
                break;
            case 'Kmi':
                command = `SET_KMI(${value})`;
                break;
            case 'Kmd':
                command = `SET_KMD(${value})`;
                break;
            case 'absolutemax_tilt':
                command = `MAX_TILT(${value})`;
                break;
            case 'absolutemin_tilt':
                command = `MIN_TILT(${value})`;
                break;
            case 'motorspeed_setpoint':
                command = `TARGET_SPEED(${value})`;
                break;
            default:
                command = `${paramName}:${value}`; // fallback
        }

        try {
            const response = await fetch('/send', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command })
            });

            const result = await response.json();
            console.log(result.status);
            document.getElementById(`${paramName}Input`).value = '';
            fetchMessages();
        } catch (e) {
            console.error(`Failed to send ${paramName}:`, e);
        }
    }


    setInterval(fetchMessages, 1000); // Refresh every second
    window.onload = fetchMessages;
</script>
</html>
"""

@app.route('/', methods=['GET'])
def home():
    return render_template_string(html_page)

@app.route('/send', methods=['POST'])
def send():
    data = request.get_json()
    command = request.get_json().get('command')
    print(f"Received command: {command}")
    sent_status = "Not sent (ESP not connected)"
    with esp_lock:
        if esp_socket:
            try:
                esp_socket.send(command.encode())
                sent_status = "Command sent to ESP"
            except Exception as e:
                sent_status = f"Failed to send: {e}"

    return jsonify({"status": sent_status})

@app.route('/messages')
def get_messages():
    with msg_lock:
        return jsonify(list(messages))

# TCP Server function
def tcp_server():
    global esp_socket
    welcome_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    welcome_socket.bind((server_ip, server_port))
    welcome_socket.listen(1)
    print(f'TCP Server running on port {server_port}')

    while True:
        connection_socket, caddr = welcome_socket.accept()
        print(f"ESP Connected from {caddr}")
        with esp_lock:
            esp_socket = connection_socket
        try:
            while True:
                try:
                    cmsg = connection_socket.recv(1024)
                    if not cmsg:
                        print(f"ESP {caddr} disconnected.")
                        break
                    with msg_lock:
                        messages.append(cmsg.decode())
                    print("Received from ESP:", cmsg.decode())
                except socket.timeout:
                    print(f"Timeout: No data received from {caddr} for 10 seconds.")
                    break  # disconnect and wait for next client
        except Exception as e:
            print(f"Error: {e}")
        finally:
            connection_socket.close()

# Start TCP server in background thread
tcp_thread = threading.Thread(target=tcp_server, daemon=True)
tcp_thread.start()



if __name__ == '__main__':
    app.run(host='192.168.219.234', port=8000, debug=False)
