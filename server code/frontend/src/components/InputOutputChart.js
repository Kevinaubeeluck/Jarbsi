import React, { useEffect, useState } from 'react';
import { Line } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Tooltip,
  Legend
} from 'chart.js';

ChartJS.register(LineElement, CategoryScale, LinearScale, PointElement, Tooltip, Legend);

const InputOutputChart = ({ messages }) => {
  const [inputs, setInputs] = useState([]);
  const [outputs, setOutputs] = useState([]);

  useEffect(() => {
    const newInputs = [];
    const newOutputs = [];

    messages.forEach(msg => {
    const inputMatch = msg.match(/input:(-?\d+(\.\d+)?)/i);
    const outputMatch = msg.match(/output:(-?\d+(\.\d+)?)/i);
      if (inputMatch) newInputs.push(parseFloat(inputMatch[1]));
      if (outputMatch) newOutputs.push(parseFloat(outputMatch[1]));
    });

    setInputs(newInputs);
    setOutputs(newOutputs);
  }, [messages]);

  const labels = inputs.map((_, idx) => idx);

  const data = {
    labels,
    datasets: [
      {
        label: 'Input',
        data: inputs,
        borderColor: 'red',
        borderDash: [5, 5],
        borderWidth: 2,
        fill: false,
      },
      {
        label: 'Output',
        data: outputs,
        borderColor: 'red',
        borderWidth: 2,
        fill: false,
      }
    ]
  };

  const options = {
    responsive: true,
    plugins: { legend: { position: 'top' } },
    scales: {
        y: {
        beginAtZero: false,     // Let negative values show
        ticks: {
            callback: function (value) {
            return value.toFixed(1);  // Optional: cleaner y-axis numbers
            }
        }
        }
    }
    };

  return (
    <div style={{ background: '#fff', padding: '15px', borderRadius: '10px', boxShadow: '0 2px 8px rgba(0,0,0,0.1)' }}>
      <h3>Input vs Output Chart</h3>
      <Line data={data} options={options} />
    </div>
  );
};

export default InputOutputChart;
