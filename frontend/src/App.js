import React, { useEffect, useState, useCallback } from 'react';
import ParameterBox from './components/ParameterBox';
import MessageLog from './components/MessageLog';

const PARAMS = [
  'Kp', 'Ki', 'Kd',
  'Kmp', 'Kmi', 'Kmd',
  'absolutemax_tilt', 'absolutemin_tilt',
  'motorspeed_setpoint'
];

function App() {
  const [values, setValues] = useState({});
  const [messages, setMessages] = useState([]);

  const fetchCurrent = useCallback((force = false) => {
    fetch(`http://localhost:8000/api/current_values${force ? '?force=true' : ''}`)
      .then(r => r.json())
      .then(data => setValues(v => ({ ...v, ...data })))
      .catch(console.error);
  }, []);

  const fetchMessages = useCallback(() => {
    fetch('http://localhost:8000/api/messages')
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
      if (e.key == 'w' || e.key == 'W') {
        const newVal = 10;
        sendParam('motorspeed_setpoint', newVal);
      }
      if(e.key == 's' || e.key == 'S') {
        const newVal = -10;
        sendParam('motorspeed_setpoint', newVal);
      }
      if(e.key == 'Control'){
        const newVal = 0;
        sendParam('motorspeed_setpoint', newVal);
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [values]);

  const sendParam = (param, value) => {
    const commands = {
      Kp: `SET_KP(${value})`,
      Ki: `SET_KI(${value})`,
      Kd: `SET_KD(${value})`,
      Kmp: `SET_KMP(${value})`,
      Kmi: `SET_KMI(${value})`,
      Kmd: `SET_KMD(${value})`,
      absolutemax_tilt: `MAX_TILT(${value})`,
      absolutemin_tilt: `MIN_TILT(${value})`,
      motorspeed_setpoint: `TARGET_SPEED(${value})`
    };
    fetch('http://localhost:8000/api/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: commands[param] })
    }).then(fetchMessages);
  };

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(250px, 1fr))', gap: '20px', padding: '20px' }}>
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
     </div>
  );
}

export default App;
