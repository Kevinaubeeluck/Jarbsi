import React from 'react';

function MessageLog({ messages }) {
  return (
    <div style={{ background: '#fff', padding: '15px', borderRadius: '10px', boxShadow: '0 2px 8px rgba(0,0,0,0.1)', maxHeight: '250px', overflowY: 'auto' }}>
      <h3>Messages</h3>
      {messages.map((m, idx) => (
        <div key={idx} style={{ fontSize: '0.9em', wordBreak: 'break-word' }}>{m}</div>
      ))}
    </div>
  );
}

export default MessageLog;
