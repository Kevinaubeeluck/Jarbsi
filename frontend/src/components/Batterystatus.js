import React, { useEffect, useState } from 'react';

function BatteryStatus() {
  const [level, setLevel] = useState(null);

  useEffect(() => {
    const fetchBattery = () => {
      fetch("http://localhost:8000/api/batteryinfo")
        .then(res => res.json())
        .then(data => setLevel(data.level))
        .catch(console.error);
    };

    fetchBattery();
    const interval = setInterval(fetchBattery, 5000); // Update every 5 sec

    return () => clearInterval(interval);
  }, []);

  const getBatteryIcon = () => {
    if (level === null) return "🔌";
    if (level >= 80) return "🔋";
    if (level >= 50) return "🔋";
    if (level >= 20) return "🪫";
  };

  return (
    <div style={{ textAlign: 'right', padding: '10px', fontSize: '1.5em' }}>
      Battery: {getBatteryIcon()} {level !== null ? `${level}%` : "Loading..."}
    </div>
  );
}

export default BatteryStatus;
