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
server_ip = '192.168.70.169'
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
        }
        .container {
            text-align: center;
            background-color: white;
            padding: 40px;
            border-radius: 15px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
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
    </style>
</head>

<div class="container">
    <h1>ESP PID values updater</h1>
    <h2>Kp,ki,kd<h2>
    <form action="/send" method="post">
        <input type="text" id="command" name="command" placeholder="Enter command here" required><br>
        <input type="submit" value="Send Command">
    </form>
    <hr>
    <h2>Messages from ESP:</h2>
    <div id="messageDisplay" style="text-align:left;max-height:200px;overflow:auto;border:1px solid #ccc;padding:10px;border-radius:8px;"></div>
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
    command = request.form.get('command')
    print(f"Received command: {command}")
    sent_status = "Not sent (ESP not connected)"
    with esp_lock:
        if esp_socket:
            try:
                esp_socket.send(command.encode())
                sent_status = "Command sent to ESP"
            except Exception as e:
                sent_status = f"Failed to send: {e}"

    return f"<p style='text-align:center;font-family:sans-serif;'>Command received: <strong>{command}</strong></p><p style='text-align:center;'><a href='/'>Go back</a></p>"

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
                cmsg = connection_socket.recv(1024)
                if not cmsg:
                    print(f"ESP {caddr} disconnected.")
                    break
                with msg_lock:
                    messages.append(cmsg.decode())
                print("Received from ESP:", cmsg.decode())
                
                # if not cmsg.isalnum():
                #     response = "Not alphanumeric."
                # else:
                #     response = "Alphanumeric"

               # connection_socket.send(response.encode())
        except Exception as e:
            print(f"Error: {e}")
        finally:
            connection_socket.close()

# Start TCP server in background thread
tcp_thread = threading.Thread(target=tcp_server, daemon=True)
tcp_thread.start()



if __name__ == '__main__':
    app.run(host='192.168.70.169', port=8000, debug=False)
