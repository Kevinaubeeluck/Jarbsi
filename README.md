# The iron man HUD (Flask + React)

## Backend (setup & run)

```bash
cd backend
python -m venv .venv
source venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python app.py
```

Flask will start on **http://localhost:8000** and also launch the background TCP server to communicate with the ESP32.

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
