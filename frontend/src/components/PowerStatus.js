import React, { useEffect, useState } from 'react';
// Step 1: Import the icons we'll need from react-icons
import {
  FaBatteryFull,
  FaBatteryThreeQuarters,
  FaBatteryHalf,
  FaBatteryQuarter,
  FaBatteryEmpty,
  FaPlug,
  FaExclamationTriangle,
  FaSpinner,
  FaSyncAlt
} from 'react-icons/fa';

const API = "http://192.168.1.37:8000";

export default function PowerStatus() {
  const [t, setT] = useState(null); // latest telemetry object
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const getT = () => {
      // Don't set loading to true on subsequent fetches to avoid UI flicker
      // setIsLoading(true); 
      setError(null);

      fetch(`${API}/api/telemetry`)
        .then(r => {
          if (!r.ok) {
            throw new Error('Failed to fetch telemetry data.');
          }
          return r.json();
        })
        .then(data => {
            setT(data);
        })
        .catch(err => {
            console.error(err);
            setError(err.message);
        })
        .finally(() => {
            // This will run after .then() or .catch()
            setIsLoading(false);
        });
    };

    getT(); // Initial fetch
    const id = setInterval(getT, 5000);
    return () => clearInterval(id);
  }, []);
  
  // Loading to make it less aggravating things dont work 
  if (isLoading) {
    return (
        <div style={containerStyle}>
            <div style={{display: 'flex', alignItems: 'center', gap: '0.6rem'}}>
                 <FaSpinner className="fa-spin" style={{fontSize:'1.4rem', color: '#a0aec0'}}/>
                 <strong>Loading Status...</strong>
            </div>
        </div>
    );
  }

const thStyle = {
  padding: '4px 0',
  fontWeight: 'normal', 
  textAlign: 'left',
};

const tdStyle = {
  padding: '4px 0',
  textAlign: 'right',
  paddingLeft: '1rem',
};

  // Error for when connection falls or anything drops 
  if (error) {
    return (
        <div style={containerStyle}>
            <div style={{display: 'flex', alignItems: 'center', gap: '0.6rem', color: '#e53e3e'}}>
                <FaExclamationTriangle style={{fontSize:'1.4rem'}}/>
                <strong>Error</strong>
            </div>
            <p style={{margin: '0.5rem 0 0', color: '#e53e3e'}}>{error}</p>
        </div>
    );
  }
  
  const pct = t.level ?? null;

  // This handles what battery icon 
  const getBatteryDisplay = () => {
    if (pct === null) return { Icon: FaPlug, color: '#38b2ac' }; 
    if (pct > 80) return { Icon: FaBatteryFull, color: '#48bb78' }; 
    if (pct > 60) return { Icon: FaBatteryThreeQuarters, color: '#48bb78' }; 
    if (pct > 35) return { Icon: FaBatteryHalf, color: '#f6e05e' }; 
    if (pct > 10) return { Icon: FaBatteryQuarter, color: '#f56565' }; 
    return { Icon: FaBatteryEmpty, color: '#c53030' }; 
  };

  const { Icon, color } = getBatteryDisplay();

  const handleSetFull = () => {
    fetch(`${API}/api/batteryinfo`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ level: 100 })
    })
    .then(r => {
        if (!r.ok) throw new Error('Failed to set battery level.');
        setT(currentT => ({ ...currentT, level: 100 }));
    })
    .catch(console.error);
  };

  return (
    <div style={containerStyle}>
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.6rem' }}>
        <Icon style={{ fontSize: '1.6rem', color: color, transition: 'color 0.3s ease' }} />
        <strong style={{fontSize: '1.1rem'}}>
          {pct === null ? 'Plugged In' :
            <>
              {`${pct.toFixed(0)}`}
              {/* The '%' is hidden from screen readers to avoid announcing 'percent' twice */}
              <span aria-hidden="true"> %</span>
              {/* This text is only read by screen readers */}
              {/* <span className="visually-hidden"> battery percentage</span> */}
            </>
          }
        </strong>
      </div>


      <table style={{ width: '100%', borderCollapse: 'collapse' }}>
        <tbody>
          <tr>
            <th scope="row" style={thStyle}>Battery Voltage</th>
            <td style={tdStyle}>{t.bat_volt?.toFixed(2) ?? '—'} V</td>
          </tr>

          <tr>
            <th scope="row" style={thStyle}>Motor Voltage</th>
            <td style={tdStyle}>{t.mot_volt?.toFixed(2) ?? '—'} V</td>
          </tr>
          <tr>
            <th scope="row" style={thStyle}>Motor Current</th>
            <td style={tdStyle}>{t.mot_curr?.toFixed(2) ?? '—'} A</td>
          </tr>
          <tr>
            <th scope="row" style={thStyle}>Motor Power</th>
            <td style={tdStyle}>{t.mot_pow?.toFixed(2) ?? '—'} A</td>
          </tr>
          <tr>
            <th scope="row" style={thStyle}>5V rail Voltage</th>
            <td style={tdStyle}>{t.v5_volt?.toFixed(2) ?? '—'} A</td>
          </tr>
          <tr>
            <th scope="row" style={thStyle}>5V rail Current</th>
            <td style={tdStyle}>{t.v5_curr?.toFixed(2) ?? '—'} V</td>
          </tr>
          <tr>
            <th scope="row" style={thStyle}>5V rail Power</th>
            <td style={tdStyle}>{t.v5_pow?.toFixed(2) ?? '—'} A</td>
          </tr>
        </tbody>
      </table>

      <button style={buttonStyle} onClick={handleSetFull}>
        <FaSyncAlt /> Set to 100%
      </button>
    </div>
  );
}


const containerStyle = {
  backgroundColor: '#2d3748',
  border: '1px solid #4a5568',
  borderRadius: 8,
  padding: '1rem',
  display: 'flex',
  flexDirection: 'column',
  gap: '0.75rem',
  color: 'white',
};

const buttonStyle = {
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  gap: '0.5rem',
  padding: '0.5rem 1rem',
  border: 'none',
  borderRadius: '6px',
  backgroundColor: '#4a5568',
  color: 'white',
  fontWeight: 'bold',
  cursor: 'pointer',
  transition: 'background-color 0.2s',
};