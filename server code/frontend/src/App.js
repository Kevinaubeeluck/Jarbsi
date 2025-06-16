import React, { useEffect, useState, useCallback, useRef } from 'react';
import ParameterBox from './components/ParameterBox';
import MessageLog from './components/MessageLog';
import Leaderboard from './components/leaderboard';
import PowerStatus from './components/PowerStatus';
import InputOutputChart from './components/InputOutputChart';


const API_BASE = "http://192.168.34.158:8000"; 

const PARAMS = [
  'direction','Kp',  'Ki', 'Kd',
  'Kmp', 'Kmi', 'Kmd',
  'absolutemax_tilt', 'absolutemin_tilt',
  'motorspeed_setpoint', 'turn', 
  'K3p', 'K3i', 'K3d'
];

function App() {
  const activeKeyRef = useRef(null);
  const [values, setValues] = useState({});
  const [messages, setMessages] = useState([]);

  const fetchCurrent = useCallback((force = false) => {
    fetch(`${API_BASE}/api/current_values${force ? '?force=true' : ''}`)
      .then(r => r.json())
      .then(data => setValues(v => ({ ...v, ...data })))
      .catch(console.error);
  }, []);

  const fetchMessages = useCallback(() => {
    fetch(`${API_BASE}/api/messages`)
      .then(r => r.json())
      .then(setMessages)
      .catch(console.error);
  }, []);

  useEffect(() => {
    fetchCurrent(true);
    fetchMessages();
    const i1 = setInterval(() => fetchCurrent(false), 1000);
    const i2 = setInterval(fetchMessages, 1000);
    return () => { clearInterval(i1); clearInterval(i2); };
  }, [fetchCurrent, fetchMessages]);


useEffect(() => {

  const handleKeyDown = (e) => {
    if (activeKeyRef.current !== e.key) {
      activeKeyRef.current = e.key;
      if (e.key === 'w' || e.key === 'W') {
        sendParam('direction', 10);
      } else if (e.key === 's' || e.key === 'S') {
        sendParam('direction', -10);
      }
      if (e.key === 'a' || e.key === 'A') {
        sendParam('turn', -6);
      }
      else if (e.key === 'd' || e.key === 'D'){
        sendParam('turn', 6);
      }
    }
  };

  const handleKeyUp = (e) => {
    if (e.key === activeKeyRef.current) {
      activeKeyRef.current = null;
      sendParam('direction', 0);
      sendParam('turn', 0);

    }
  };

  window.addEventListener('keydown', handleKeyDown);
  window.addEventListener('keyup', handleKeyUp);

  return () => {
    window.removeEventListener('keydown', handleKeyDown);
    window.removeEventListener('keyup', handleKeyUp);
  };
}, []);


  const sendParam = (param, value) => {
    const commands = {
      direction: `SET_DIR(${value})`,
      Kp: `SET_KP(${value})`,
      Ki: `SET_KI(${value})`,
      Kd: `SET_KD(${value})`,
      Kmp: `SET_KMP(${value})`,
      Kmi: `SET_KMI(${value})`,
      Kmd: `SET_KMD(${value})`,
      K3p: `SET_K3P(${value})`,
      K3i: `SET_K3I(${value})`,
      K3d: `SET_K3D(${value})`,
      absolutemax_tilt: `MAX_TILT(${value})`,
      absolutemin_tilt: `MIN_TILT(${value})`,
      motorspeed_setpoint: `bias(${value})`,
      turn: `SET_TURN(${value})`
    };
    fetch(`${API_BASE}/api/send`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: commands[param] })
    }).then(fetchMessages);
  };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(250px, 1fr))', gap: '20px', padding: '20px' }}>
      <div style={{ gridColumn: '1 / -1' }}>
        <PowerStatus />
      </div>
      {PARAMS.map(p => (
        <ParameterBox
          key={p}
          name={p}
          current={values[p]}
          onSend={val => sendParam(p, val)}
        />
      ))}
      <div style={{ gridColumn: '1 / -1' }}>
        <MessageLog messages={messages} />
      </div>
      <div style={{ gridColumn: '1 / -1' }}>
        <Leaderboard />
      </div>
      <div style={{ gridColumn: '1 / -1' }}>
        <InputOutputChart messages={messages} />
      </div>
     </div>
  );
}

export default App;