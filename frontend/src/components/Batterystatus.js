import React, { useEffect, useState } from 'react';

const API = "http://192.168.140.235:8000";

export default function BatteryStatus() {
  const [level, setLevel] = useState(null);

  // poll server every 5 s
  useEffect(() => {
    const fetchBattery = () =>
      fetch(`${API}/api/batteryinfo`)
        .then(r => r.json())
        .then(({ level }) => setLevel(level))
        .catch(console.error);

    fetchBattery();
    const id = setInterval(fetchBattery, 5000);
    return () => clearInterval(id);
  }, []);

  // POST 100 % to the API
  const setToFull = () =>
    fetch(`${API}/api/batteryinfo`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ level: 100 })
    }).then(() => setLevel(100)); // optimistic

  const icon = (() => {
    if (level === null)      return "🔌";      // unknown / charging
    if (level >= 80)         return "🔋";      // full
    if (level >= 50)         return "🔋︎";     // three-quarters (alt glyph or text)
    if (level >= 20)         return "🪫";      // low
    return "❗";                              // critically low
  })();

  return (
    <div style={{
      display: 'flex',
      gap: '0.6rem',
      alignItems: 'center',
      padding: '1rem',
      border: '1px solid #ddd',
      borderRadius: '8px'
    }}>
      <span style={{ fontSize: '1.4rem' }}>{icon}</span>
      <span>{level === null ? '—' : `${level.toFixed(0)} %`}</span>

      <button
        style={{
          marginLeft: 'auto',
          padding: '0.4rem 0.8rem',
          borderRadius: '6px',
          border: 'none',
          background: '#4caf50',
          color: '#fff',
          cursor: 'pointer'
        }}
        onClick={setToFull}
      >
        Set 100 %
      </button>
    </div>
  );
}
