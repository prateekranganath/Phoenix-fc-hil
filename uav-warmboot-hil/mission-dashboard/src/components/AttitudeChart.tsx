import { CartesianGrid, Legend, Line, LineChart, XAxis, YAxis } from "recharts";
import type { AttitudeSample } from "../types/telemetry";

const LINES = [
  { key: "roll", name: "Roll (truth)", color: "#3fb6d9", dash: undefined },
  { key: "pitch", name: "Pitch (truth)", color: "#f5a524", dash: undefined },
  { key: "estRoll", name: "Roll (MCU est.)", color: "#3fb6d9", dash: "4 3" },
  { key: "estPitch", name: "Pitch (MCU est.)", color: "#f5a524", dash: "4 3" },
];

/** Scrolling roll/pitch [deg]: solid = simulator truth, dashed = what the MCU reported. Dashed lines stall during an outage. */
export function AttitudeChart({ data }: { data: AttitudeSample[] }) {
  return (
    <section className="panel">
      <h2>Attitude Recovery</h2>
      <LineChart width={520} height={240} data={data}>
        <CartesianGrid stroke="#2a3441" />
        <XAxis dataKey="t" type="number" domain={["dataMin", "dataMax"]} tickFormatter={(v) => `${v.toFixed(0)}s`} stroke="#8b96a5" />
        <YAxis unit="°" stroke="#8b96a5" domain={[-10, 10]} allowDataOverflow={false} />
        <Legend />
        {LINES.map((l) => (
          <Line key={l.key} dataKey={l.key} name={l.name} stroke={l.color} strokeDasharray={l.dash} dot={false} isAnimationActive={false} />
        ))}
      </LineChart>
    </section>
  );
}
