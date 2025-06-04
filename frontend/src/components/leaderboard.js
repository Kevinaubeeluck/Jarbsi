// src/components/Leaderboard.js
import React, { useEffect, useState } from 'react';

function Leaderboard() {
  const [entries, setEntries] = useState([]);

  useEffect(() => {
    const fetchLeaderboard = () => {
        fetch("http://localhost:8000/api/leaderboard")
        .then(res => res.json())
        .then(data => setEntries(data))
        .catch(console.error);
    };
  
    fetchLeaderboard();
    const interval = setInterval(fetchLeaderboard, 2000);

    return () => clearInterval(interval);
    },[]);

  return (
    <div style={{ padding: '20px' }}>
      <h2>Leaderboard</h2>
      <table style={{ width: '100%', borderCollapse: 'collapse' }}>
        <thead>
          <tr>
            <th style={{ borderBottom: '2px solid #ccc' }}>Name</th>
            <th style={{ borderBottom: '2px solid #ccc' }}>Score</th>
          </tr>
        </thead>
        <tbody>
          {entries.map((entry, idx) => (
            <tr key={idx}>
              <td style={{ padding: '8px', borderBottom: '1px solid #eee' }}>{entry.name}</td>
              <td style={{ padding: '8px', borderBottom: '1px solid #eee' }}>{entry.score}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

export default Leaderboard;
