/** Mirrors Sim.snapshot() in sim-engine/server.py. World frame is NED, quaternion is [w,x,y,z] body->NED. */
export type BootPath = "NONE" | "COLD" | "WARM_FULL" | "WARM_PARTIAL";

export interface Metrics {
  active: boolean;
  settled: boolean;
  downtime_ms: number | null;
  time_to_control_ms: number | null;
  max_tilt_deg: number;
  settling_ms: number | null;
  drift_deg: number;
  max_altitude_drop_m: number;
  t0: number | null; // sim time of the fault, for live elapsed readouts
}

export interface Telemetry {
  t: number;
  pos: [number, number, number];
  altitude_m: number;
  quat: [number, number, number, number];
  euler: [number, number, number]; // roll, pitch, yaw [deg], ground truth
  est_euler: [number, number, number]; // as reported by the MCU
  motors: [number, number, number, number];
  mcu: { online: boolean; ctrl_active: boolean; boot_path: BootPath };
  metrics: Metrics;
  overruns: number;
}

export type FaultCmd = "iwdg_hang" | "wwdg_overrun" | "torn_write" | "cold_boot";

/** One chart sample (truth vs MCU estimate). */
export interface AttitudeSample {
  t: number;
  roll: number;
  pitch: number;
  estRoll: number;
  estPitch: number;
}
