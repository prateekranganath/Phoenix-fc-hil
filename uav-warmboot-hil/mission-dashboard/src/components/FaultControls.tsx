import { useState } from "react";
import type { FaultCmd } from "../types/telemetry";

const BUTTONS: { cmd: FaultCmd; label: string }[] = [
  { cmd: "iwdg_hang", label: "IWDG Hang" },
  { cmd: "wwdg_overrun", label: "WWDG Overrun" },
  { cmd: "torn_write", label: "Torn Write" },
  { cmd: "cold_boot", label: "Force Cold Boot" },
];

/** Fault injection. The outage slider adds extra down-time before the MCU starts rebooting. */
export function FaultControls({ onFault, disabled }: { onFault: (c: FaultCmd, outageMs: number) => void; disabled: boolean }) {
  const [outage, setOutage] = useState(0);
  return (
    <section className="panel">
      <h2>Fault Injection</h2>
      <div className="btn-row">
        {BUTTONS.map((b) => (
          <button key={b.cmd} disabled={disabled} onClick={() => onFault(b.cmd, outage)}>
            {b.label}
          </button>
        ))}
      </div>
      <label className="slider">
        Outage delay: <b>{outage} ms</b>
        <input type="range" min={0} max={3000} step={50} value={outage} onChange={(e) => setOutage(+e.target.value)} />
      </label>
    </section>
  );
}
