import React, { useState, useEffect } from 'react';
import { FaVideo, FaExclamationTriangle, FaSpinner } from 'react-icons/fa';

const VideoStream = ({ apiBase, onManualOverride, isManualOverrideActive }) => {
  const [imageSrc, setImageSrc] = useState('');
  const [streamError, setStreamError] = useState(false);

  const handleImageError = () => {
      setStreamError(true);
      setImageSrc('');
  };


  useEffect(() => {
    const intervalId = setInterval(() => {
      const newSrc = `${apiBase}/api/latest_frame?_=${new Date().getTime()}`;
      setImageSrc(newSrc);
    }, 50); // This refreshes at 5 frames per second(max from pi is 2 so 5 more than enough)

    return () => clearInterval(intervalId);
  }, [apiBase]);


  const [latestObstacleMsg, setLatestObstacleMsg] = useState('');

useEffect(() => {
  const fetchObstacleMessage = () => {
    fetch(`${apiBase}/api/latest_obstacles`)
      .then(res => res.json())
      .then(data => {
        // Check if the response is a plain string or inside a 'message' key
        const msg = typeof data === 'string' ? data : data.message;
        setLatestObstacleMsg(msg);
      })
      .catch(() => setLatestObstacleMsg(''));
  };

  const interval = setInterval(fetchObstacleMessage, 1000);
  return () => clearInterval(interval);
}, [apiBase]);
  
  const containerStyle = {
    backgroundColor: '#2d3748',
    padding: '1rem',
    borderRadius: '8px',
    boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
    color: 'white',
  };

  const imageStyle = {
    width: '100%',
    aspectRatio: '16 / 9',
    objectFit: 'cover',
    borderRadius: '4px',
    backgroundColor: '#1a202c',
  };

  const buttonStyle = {
    padding: '0.75rem 1.5rem',
    border: 'none',
    borderRadius: '6px',
    backgroundColor: isManualOverrideActive ? '#e53e3e' : '#2d3748',
    color: 'white',
    fontWeight: 'bold',
    cursor: 'pointer',
    textAlign: 'center',
    width: '100%',
    transition: 'background-color 0.2s',
    marginTop: '1rem',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    gap: '0.5rem',
    animation: isManualOverrideActive ? 'pulse 1.5s infinite' : 'none',
  };
  
  const keyframes = `
    @keyframes pulse {
      0% {
        box-shadow: 0 0 0 0 rgba(229, 62, 62, 0.7);
      }
      70% {
        box-shadow: 0 0 0 10px rgba(229, 62, 62, 0);
      }
      100% {
        box-shadow: 0 0 0 0 rgba(229, 62, 62, 0);
      }
    }
  `;

  return (
    <div style={containerStyle}>
      <style>{keyframes}</style>
      <h2 style={{ marginTop: 0, marginBottom: '1rem', display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
        <FaVideo aria-hidden="true" /> Live Feed
      </h2>
      {imageSrc ? (
        <img
          src={imageSrc}
          alt="Live video feed from Raspberry Pi"
          style={imageStyle}
          onError={handleImageError}
        />

      ) : (
        <div style={{...imageStyle, display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', gap: '1rem'}}>
          <FaSpinner aria-hidden="true" className="fa-spin" style={{ fontSize: '2rem' }} />
          <p style={{ color: '#a0aec0' }}>{streamError ? 'Video stream disconnected.' : 'Connecting to video stream...'}</p>
        </div>
      )}
      <button
        style={buttonStyle}
        onClick={onManualOverride}
        onMouseOver={e => e.currentTarget.style.backgroundColor = isManualOverrideActive ? '#c53030' : '#4a5568'}
        onMouseOut={e => e.currentTarget.style.backgroundColor = isManualOverrideActive ? '#e53e3e' : '#2d3748'}
      >
        <FaExclamationTriangle aria-hidden="true" /> {isManualOverrideActive ? 'Deactivate Manual Override' : 'Activate Manual Override'}
      </button>
      <p style={{ marginTop: '1rem', color: '#FF0000', fontSize: '1.5rem', textAlign: 'center' }}>
  {latestObstacleMsg || 'No obstacle data.'}
</p>

    </div>


  );
};

export default VideoStream;