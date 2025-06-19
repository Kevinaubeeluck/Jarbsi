# The iron man HUD (Flask + React)

## Backend (run)

```bash
cd Jarbsi
source venv/bin/activate   # Windows: .venv\Scripts\activate
python app.py
```

Flask will start on **http://localhost:8000** and also launch the background TCP server to communicate with the ESP32.

## Frontend (run)

```bash
cd frontend
nohup npm start > /dev/null 2>&1 &
```

The React dev‑server runs on **http://localhost:3000** and proxies API calls to the Flask backend (CORS is enabled).

## Build for production

```bash
npm run build          # creates static files in frontend/build
```

You can then serve them with any static host or copy them into the Flask app if you prefer a single‑server deployment.
