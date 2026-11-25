import { useTelemetry } from "./hooks/useTelemetry";
import { DroneViewport3D } from "./components/DroneViewport3D";
import { FaultControls } from "./components/FaultControls";
import { RecoveryMetrics } from "./components/RecoveryMetrics";
import { AttitudeChart } from "./components/AttitudeChart";

export default function App() {
  const { latest, history, connected, sendFault } = useTelemetry();
  return (
    <div className="layout">
      <header>
        UAV Warm-Boot Mission Control
        <span className={connected ? "link up" : "link down"}>{connected ? "● server connected" : "○ server offline"}</span>
      </header>
      <div className="viewport"><DroneViewport3D t={latest} /></div>
      <aside>
        <RecoveryMetrics t={latest} />
        <FaultControls onFault={sendFault} disabled={!connected} />
        <AttitudeChart data={history} />
      </aside>
    </div>
  );
}
