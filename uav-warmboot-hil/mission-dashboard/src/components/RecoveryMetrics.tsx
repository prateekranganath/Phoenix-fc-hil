import type { BootPath, Telemetry } from "../types/telemetry";

const BADGE: Record<BootPath, { label: string; color: string }> = {
  NONE: { label: "NO FAULT", color: "#5b6572" },
  COLD: { label: "COLD", color: "#e5484d" },
  WARM_FULL: { label: "WARM-FULL", color: "#30a46c" },
  WARM_PARTIAL: { label: "WARM-PARTIAL", color: "#f5a524" },
};

/** Value while measuring shows the live elapsed time since the fault; "-" if there is no fault yet. */
function ms(v: number | null, live: number | null) {
  if (v !== null) return `${v.toFixed(0)} ms`;
  return live !== null ? `${live.toFixed(0)} ms…` : "-";
}

export function RecoveryMetrics({ t }: { t: Telemetry | null }) {
  if (!t) return <section className="panel"><h2>Recovery</h2>waiting for telemetry…</section>;
  const m = t.metrics;
  const live = m.active && m.t0 !== null ? (t.t - m.t0) * 1000 : null;
  const badge = BADGE[t.mcu.boot_path];
  const mcuState = !t.mcu.online ? "OFFLINE" : t.mcu.ctrl_active ? "CONTROLLING" : "ALIVE (not flying)";
  const rows: [string, string][] = [
    ["Downtime", ms(m.downtime_ms, live)],
    ["Time-to-Control", ms(m.time_to_control_ms, live)],
    ["Settling Time", ms(m.settling_ms, m.settled ? null : live)],
    ["Max Tilt", `${m.max_tilt_deg.toFixed(1)}°`],
    ["Attitude Drift", `${m.drift_deg.toFixed(1)}°`],
    ["Altitude", `${t.altitude_m.toFixed(1)} m`],
    ["Altitude Drop", `${m.max_altitude_drop_m.toFixed(1)} m`],
  ];
  return (
    <section className="panel">
      <h2>Recovery</h2>
      <div className="badge" style={{ background: badge.color }}>{badge.label}</div>
      <div className="mcu-state">MCU: {mcuState}</div>
      <dl>
        {rows.map(([k, v]) => (
          <div key={k}><dt>{k}</dt><dd>{v}</dd></div>
        ))}
      </dl>
    </section>
  );
}
