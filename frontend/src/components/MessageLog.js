import React, { useEffect, useRef } from 'react';

function MessageLog({ messages }) {
  // Create a ref to hold a reference to the log container's DOM element.
  const logContainerRef = useRef(null);

  // This useEffect hook runs every time the 'messages' array changes.
  useEffect(() => {
    // If the ref is attached to the element, scroll to the bottom.
    if (logContainerRef.current) {
      const { scrollHeight, clientHeight } = logContainerRef.current;
      logContainerRef.current.scrollTop = scrollHeight - clientHeight;
    }
  }, [messages]); // The dependency array ensures this runs only when messages update.

  return (
    // Attach the ref to the main scrollable div.
    <div 
      ref={logContainerRef} 
      style={{ 
        background: '#2d3748', 
        padding: '15px', 
        color: 'white', 
        borderRadius: '10px', 
        boxShadow: '0 2px 8px rgba(0,0,0,0.1)', 
        height: '250px', // A fixed height is necessary for scrolling
        overflowY: 'auto' 
      }}
    >
      <h3 style={{ marginTop: 0, borderBottom: '1px solid #4a5568', paddingBottom: '10px' }}>Messages</h3>
      {messages.map((m, idx) => (
        <div key={idx} style={{ fontSize: '0.9em', wordBreak: 'break-word', paddingTop: '4px' }}>
          {m}
        </div>
      ))}
    </div>
  );
}

export default MessageLog;
