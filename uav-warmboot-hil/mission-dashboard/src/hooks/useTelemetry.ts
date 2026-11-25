import { useCallback, useEffect, useRef, useState } from "react";
import type { AttitudeSample, FaultCmd, Telemetry } from "../types/telemetry";

const WS_URL = "ws://127.0.0.1:8000/ws";
const HISTORY_S = 15;
const CHART_HZ = 20; // chart re-renders at this rate; samples arrive at 60 Hz

/** Live telemetry over WebSocket (auto-reconnect) plus a rolling attitude history for the chart. */
export function useTelemetry() {
  const [latest, setLatest] = useState<Telemetry | null>(null);
  const [history, setHistory] = useState<AttitudeSample[]>([]);
  const [connected, setConnected] = useState(false);
  const ws = useRef<WebSocket | null>(null);
  const buf = useRef<AttitudeSample[]>([]);

  useEffect(() => {
    let closed = false;
    let retry: number;
    const connect = () => {
      const s = new WebSocket(WS_URL);
      ws.current = s;
      s.onopen = () => setConnected(true);
      s.onclose = () => {
        setConnected(false);
        if (!closed) retry = window.setTimeout(connect, 1000);
      };
      s.onmessage = (e) => {
        const m: Telemetry = JSON.parse(e.data);
        setLatest(m);
        const b = buf.current;
        if (b.length && m.t < b[b.length - 1].t) b.length = 0; // server restarted
        b.push({ t: m.t, roll: m.euler[0], pitch: m.euler[1], estRoll: m.est_euler[0], estPitch: m.est_euler[1] });
        while (b[0] && m.t - b[0].t > HISTORY_S) b.shift();
      };
    };
    connect();
    const chart = window.setInterval(() => setHistory([...buf.current]), 1000 / CHART_HZ);
    return () => {
      closed = true;
      clearTimeout(retry);
      clearInterval(chart);
      ws.current?.close();
    };
  }, []);

  const sendFault = useCallback((cmd: FaultCmd, outage_ms: number) => {
    ws.current?.send(JSON.stringify({ cmd, outage_ms }));
  }, []);

  return { latest, history, connected, sendFault };
}
