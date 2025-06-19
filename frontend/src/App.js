import React, { useEffect, useState, useCallback, useRef } from 'react';

import VideoStream from './components/VideoStream';
import ParameterBox from './components/ParameterBox';
import MessageLog from './components/MessageLog';
import KeyIndicator from './components/Keyinput'
import InputOutputChart from './components/InputOutputChart';
import PowerStatus from './components/PowerStatus';


const API_BASE = "http://192.168.145.234:8000";

const PARAMS = [
  //'Kp', 'Ki', 'Kd',
  //'Kmp', 'Kmi', 'Kmd',
  //'absolutemax_tilt', 'absolutemin_tilt',
  'bias'//, 'turn', 'direction',
 // 'K3p', 'K3i', 'K3d'
];

function App() {
  const activeKeyRef = useRef(null);
  const audioRef = useRef(null); 
  const [values, setValues] = useState({});
  const [messages, setMessages] = useState([]);
  const [isManualOverrideActive, setIsManualOverrideActive] = useState(false);

  const playSound = useCallback((soundFile) => {
    if (audioRef.current) {
      audioRef.current.pause();
      audioRef.current.currentTime = 0;
    }
    const newAudio = new Audio(`/assets/${soundFile}`);
    newAudio.play().catch(error => console.error("Audio playback failed:", error));
    audioRef.current = newAudio;
  }, []);

  const fetchMessages = useCallback(() => {
    fetch(`${API_BASE}/api/messages`)
      .then(r => r.json())
      .then(setMessages)
      .catch(console.error);
  }, []);

  const sendParam = useCallback((param, value) => {
    const commands = {
      Kp: `SET_KP(${value})`, Ki: `SET_KI(${value})`, Kd: `SET_KD(${value})`,
      K3p: `SET_K3P(${value})`, K3i: `SET_K3I(${value})`, K3d: `SET_K3D(${value})`,
      Kmp: `SET_KMP(${value})`, Kmi: `SET_KMI(${value})`, Kmd: `SET_KMD(${value})`,
      absolutemax_tilt: `MAX_TILT(${value})`, absolutemin_tilt: `MIN_TILT(${value})`,
      bias: `bias(${value})`, turn: `SET_TURN(${value})`, direction: `SET_DIR(${value})`,
      manual_override: `MANUAL_OVERRIDE(${value})`
    };
    fetch(`${API_BASE}/api/send`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: commands[param] })
    }).then(fetchMessages);
  }, [fetchMessages]); 

  const fetchPID = useCallback((force = false) => {
    fetch(`${API_BASE}/api/current_values${force ? '?force=true' : ''}`)
      .then(r => r.json())
      .then(data => {
        setValues(v => ({ ...v, ...data }));
        if (data.manual_override !== undefined) {
            setIsManualOverrideActive(data.manual_override === 1);
        }
      })
      .catch(console.error);
  }, []);

  useEffect(() => {
    fetchPID(true);
    fetchMessages();
    const i1 = setInterval(() => fetchPID(false), 5000);
    const i2 = setInterval(fetchMessages, 1000);
    return () => { clearInterval(i1); clearInterval(i2); };
  }, [fetchPID, fetchMessages]);

  useEffect(() => {
    const handleKeyDown = (e) => {
      const controlKeys = ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];
      if (controlKeys.includes(e.key)) {
        e.preventDefault();
      }

      if (activeKeyRef.current !== e.key && controlKeys.includes(e.key)) {
        activeKeyRef.current = e.key;

        if (e.key === 'ArrowUp') {
          sendParam('direction', 10);
        } else if (e.key === 'ArrowDown') {
          sendParam('direction', -10);
        } else if (e.key === 'ArrowLeft') {
          sendParam('turn', 13);
        } else if (e.key === 'ArrowRight') {
          sendParam('turn', -13);
        }
      }
    };

    const handleKeyUp = (e) => {
      if (e.key === activeKeyRef.current) {
        activeKeyRef.current = null;
        if (audioRef.current) {
            audioRef.current.pause();
            audioRef.current.currentTime = 0;
        }
        
        if (e.key === 'ArrowUp' || e.key === 'ArrowDown') {
            sendParam('direction', 0);
        }
        if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
            sendParam('turn', 0);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [isManualOverrideActive, playSound, sendParam]); 

  const handleManualOverride = () => {
    
    if (isManualOverrideActive) {
      playSound('JETWATCHTHIS.mp3');
    } else {
      playSound('KAYOUltDowned4.mp3');
    }
    const newValue = isManualOverrideActive ? 0 : 1;
    sendParam('manual_override', newValue);
    setIsManualOverrideActive(newValue === 1);
  };

  
  return (
    <main style={{ display: 'flex', flexDirection: 'row', gap: '20px', padding: '20px',color: 'white' , backgroundColor: '#1a202c', minHeight: '100vh' }}>
      {/* Left Section */}
      <section style={{ flex: '3', display: 'flex', flexDirection: 'column', gap: '20px' }} aria-labelledby="main-content-heading">
        <h1 id="main-content-heading" className="visually-hidden">Jarbis control centre </h1>
        <section aria-label="Video Stream">
          <VideoStream
            apiBase={API_BASE}
            onManualOverride={handleManualOverride}
            isManualOverrideActive={isManualOverrideActive}
          />
        </section>
        <section aria-label="IO chart">
          <InputOutputChart messages={messages} />
        </section>  
      </section>

      {/* Right Section (Sidebar) */}
      <aside style={{ flex: '2', display: 'flex', flexDirection: 'column', gap: '20px' }} aria-labelledby="sidebar-heading">
        <h2 id="sidebar-heading" className="visually-hidden">Controls and Status</h2>
        <section aria-label="Power Status">
          <PowerStatus />
        </section>
        <section aria-label="System Parameters">
            <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(220px, 1fr))', gap: '15px' }}>
                {PARAMS.map(p_name => (
                    <ParameterBox
                        key={p_name}
                        name={p_name}
                        current={values[p_name]}
                        onSend={val => sendParam(p_name, val)}
                    />
                ))}
            </div>
        </section>
        <section aria-label="Directional Keys">
          <KeyIndicator />
        </section>
        <section aria-label="Message Log" aria-live="polite">
          <MessageLog messages={messages} />
        </section>
      </aside>
    </main>
  );
}

export default App;