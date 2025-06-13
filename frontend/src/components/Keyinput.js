import React, { useState, useEffect } from 'react';

export default function KeyIndicator() {
  const [keysDown, setKeysDown] = useState(new Set());

  useEffect(() => {
    const handleKeyDown = (e) => {
      setKeysDown(prev => new Set(prev).add(e.key.toLowerCase()));
    };

    const handleKeyUp = (e) => {
      setKeysDown(prev => {
        const updated = new Set(prev);
        updated.delete(e.key.toLowerCase());
        return updated;
      });
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const keys = ['Arrowup', 'a', 's', 'd'];
  const keyStyle = (k) => ({
    width: '40px',
    height: '40px',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    margin: '5px',
    fontSize: '1.2rem',
    borderRadius: '6px',
    border: '2px solid #aaa',
    backgroundColor: keysDown.has(k) ? '#007bff' : '#f0f0f0',
    color: keysDown.has(k) ? '#fff' : '#333',
    transition: 'background 0.1s',
  });

  return (
    <div style={{ display: 'inline-flex', flexDirection: 'column', alignItems: 'center', margin: '10px' }}>
      <div style={{ display: 'flex', justifyContent: 'center' }}>
        <div style={keyStyle('w')}>↑</div>
      </div>
      <div style={{ display: 'flex', justifyContent: 'center' }}>
        <div style={keyStyle('a')}>←</div>
        <div style={keyStyle('s')}>↓</div>
        <div style={keyStyle('d')}>→</div>
      </div>
    </div>
  );
}
