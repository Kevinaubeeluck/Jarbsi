import React, { useEffect, useState } from "react";
import axios from "axios";
import { Card, CardContent } from "@/components/ui/card";
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip } from "recharts";

// Poll interval (ms)
const POLL_MS = 1000;

/**
 * <TelemetryPanel />
 *
 * Fetches /api/telemetry once a second and displays:
 *   • A card for every key‒value pair coming from the ESP
 *   • A live 60‑second line chart for BAT (battery %)
 */
export default function TelemetryPanel() {
  const [telemetry, setTelemetry] = useState({});         // { key: value }
  const [batteryHistory, setBatteryHistory] = useState([]); // [{ ts, BAT }]

  useEffect(() => {
    const fetchTelemetry = async () => {
      try {
        const res = await axios.get("/api/telemetry");
        setTelemetry(res.data);

        if (res.data.BAT !== undefined) {
          setBatteryHistory((hist) => [
            ...hist.slice(-59),                // keep last 60 points
            { ts: Date.now(), BAT: res.data.BAT },
          ]);
        }
      } catch (err) {
        // eslint-disable-next-line no-console
        console.error("Failed to fetch telemetry", err);
      }
    };

    fetchTelemetry();
    const id = setInterval(fetchTelemetry, POLL_MS);
    return () => clearInterval(id);
  }, []);

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
      {Object.entries(telemetry).map(([key, value]) => (
        <Card key={key} className="p-4 animate-fade-in">
          <CardContent>
            <p className="text-sm text-muted-foreground uppercase tracking-wide">{key}</p>
            <p className="text-2xl font-semibold tabular-nums">{value.toFixed(2)}</p>
          </CardContent>
        </Card>
      ))}

      {batteryHistory.length > 1 && (
        <Card className="col-span-full p-4 animate-fade-in">
          <CardContent>
            <h2 className="mb-2 text-lg font-medium">Battery % (last minute)</h2>
            <LineChart width={600} height={250} data={batteryHistory}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis
                dataKey="ts"
                tickFormatter={(ts) => new Date(ts).toLocaleTimeString()}
                minTickGap={30}
              />
              <YAxis domain={[0, 100]} tickFormatter={(v) => `${v}%`} />
              <Tooltip
                labelFormatter={(ts) => new Date(ts).toLocaleTimeString()}
                formatter={(v) => `${v.toFixed(2)}%`}
              />
              <Line type="monotone" dataKey="BAT" strokeWidth={2} dot={false} />
            </LineChart>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
