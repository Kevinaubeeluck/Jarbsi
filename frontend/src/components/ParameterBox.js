import React, { useState } from 'react';

function ParameterBox({ name, current, onSend }) {
  const [input, setInput] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (input !== '') {
      onSend(parseFloat(input));
      setInput('');
    }
  };

  return (
    <div
      style={{
        background: '#f9f9f9',
        padding: '15px',
        borderRadius: '10px',
        boxShadow: '0 2px 8px rgba(139, 11, 11, 0.1)',
        color: 'black'
      }}
    >
      <h3>{name}</h3>

      <form onSubmit={handleSubmit}>
        <input
          value={input}
          onChange={e => setInput(e.target.value)}
          placeholder={`Enter ${name}`}
          style={{ width: '100%', padding: '8px', marginBottom: '10px' }}
        />
        <button
          type="submit"
          style={{
            width: '100%',
            padding: '8px',
            background: '#007bff',
            color: '#fff',
            border: 'none',
            borderRadius: '6px',
          }}
        >
          Send
        </button>
      </form>

      <p style={{ marginTop: '10px' }}>
        Current:{' '}
        {current !== undefined && current !== null ? current.toFixed(3) : '?'}
      </p>
    </div>
  );
}

export default ParameterBox;
