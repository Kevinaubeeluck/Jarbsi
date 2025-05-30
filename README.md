# IoT Controller (Flask + React)

This project splits the **Flask backend** (TCP & REST API) and the **React frontend** (parameter UI with keystroke detection).

## Prerequisites
* Python 3.10+
* Node.js 16+

## Backend (setup & run)

```bash
cd backend
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python app.py
```

Flask will start on **http://localhost:8000** and also launch the background TCP server to communicate with your ESP32.

## Frontend (setup & run)

```bash
cd ../frontend
npm install
npm start
```

The React dev‑server runs on **http://localhost:3000** and proxies API calls to the Flask backend (CORS is enabled).

## Build for production

```bash
npm run build          # creates static files in frontend/build
```

You can then serve them with any static host or copy them into the Flask app if you prefer a single‑server deployment.

## Environment variables
* **TCP_IP** – IP address to bind the Python TCP server (default: 192.168.219.234)
* **TCP_PORT** – TCP port (default: 12000)

Example:

```bash
export TCP_IP=0.0.0.0
export TCP_PORT=12000
```

## Keystroke example
* **Arrow Up** increments `Kp` by **+1** (see `src/App.js`).  
  Extend `handleKeyDown` for more shortcuts.

Enjoy! 🚀
