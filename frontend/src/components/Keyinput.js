import React, { useState, useEffect } from 'react';
import { FaArrowUp, FaArrowLeft, FaArrowDown, FaArrowRight } from 'react-icons/fa';

export default function KeyIndicator() {
  const [keysDown, setKeysDown] = useState(new Set());

  useEffect(() => {
    // This component's listeners are for the visual indicator only.
    const handleKeyDown = (e) => {
      // Listen for Arrow keys
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        setKeysDown(prev => new Set(prev).add(e.key));
      }
    };

    const handleKeyUp = (e) => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        setKeysDown(prev => {
          const updated = new Set(prev);
          updated.delete(e.key);
          return updated;
        });
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  // --- Styles ---
  const containerStyle = {
    backgroundColor: '#2d3748',
    border: '1px solid #4a5568',
    borderRadius: '8px',
    padding: '1rem',
    color: 'white',
  };

  const keyGridStyle = {
    display: 'grid',
    gridTemplateColumns: 'repeat(3, 45px)',
    gridTemplateRows: 'repeat(2, 45px)',
    gap: '8px',
    justifyContent: 'center',
  };
  
  const keyStyle = (keyName) => ({
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: '1.2rem',
    fontWeight: 'bold',
    borderRadius: '6px',
    border: '2px solid #718096',
    backgroundColor: keysDown.has(keyName) ? '#4299e1' : '#1a202c',
    color: 'white',
    transition: 'background-color 0.1s ease-in-out',
    // Position the arrow keys in a standard d-pad layout
    gridColumn: keyName === 'ArrowUp' || keyName === 'ArrowDown' ? '2' : keyName === 'ArrowLeft' ? '1' : '3',
    gridRow: keyName === 'ArrowUp' ? '1' : '2',
  });

  return (
    <div style={containerStyle}>
      <h3 style={{ marginTop: 0, marginBottom: '1rem', textAlign: 'center' }}>Controls</h3>
      <div style={keyGridStyle}>
        <div style={keyStyle('ArrowUp')}><FaArrowUp /></div>
        <div style={keyStyle('ArrowLeft')}><FaArrowLeft /></div>
        <div style={keyStyle('ArrowDown')}><FaArrowDown /></div>
        <div style={keyStyle('ArrowRight')}><FaArrowRight /></div>
      </div>
    </div>
  );
}
