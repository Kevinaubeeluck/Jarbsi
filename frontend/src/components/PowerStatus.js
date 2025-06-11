import React, { useEffect, useState } from 'react';
const API = "http://192.168.17.234:8000";

export default function PowerStatus() {
  const [t, setT] = useState(null);             // latest telemetry object

  useEffect(() => {
    const getT = () =>
      fetch(`${API}/api/telemetry`)
        .then(r => r.json())
        .then(setT)
        .catch(console.error);

    getT();
    const id = setInterval(getT, 5000);
    return () => clearInterval(id);
  }, []);

  if (!t) return null;                          // nothing yet

  const pct  = t.level ?? null;
  const icon = pct === null ? "🔌"
            : pct >= 80   ? "🔋"
            : pct >= 50   ? "🔋︎"
            : pct >= 20   ? "🪫"
            : "❗";

  return (
    <div style={{
      border:'1px solid #ddd', borderRadius:8, padding:'1rem',
      display:'flex', flexDirection:'column', gap:'0.5rem'
    }}>
      <div style={{display:'flex', alignItems:'center', gap:'0.6rem'}}>
        <span style={{fontSize:'1.4rem'}}>{icon}</span>
        <strong>{pct === null ? '—' : `${pct.toFixed(0)} %`}</strong>
      </div>

      <table style={{width:'100%'}}>
        <tbody>
          <tr><td>Battery V</td><td>{t.bat_volt?.toFixed(2) ?? '—'}</td></tr>
          <tr><td>5 V rail V</td><td>{t.v5_volt?.toFixed(2) ?? '—'}</td></tr>
          <tr><td>Motor V</td><td>{t.mot_volt?.toFixed(2) ?? '—'}</td></tr>
          <tr><td>Motor A</td><td>{t.mot_curr?.toFixed(2) ?? '—'}</td></tr>
          <tr><td>5 V rail A</td><td>{t.v5_curr?.toFixed(2) ?? '—'}</td></tr>
        </tbody>
      </table>

      <button onClick={()=>{
            fetch(`${API}/api/batteryinfo`,{
              method:'POST',
              headers:{'Content-Type':'application/json'},
              body:JSON.stringify({level:100})
            }).then(()=>setT(t=>({...t, level:100})));
          }}>
        Set 100 %
      </button>
    </div>
  );
}
