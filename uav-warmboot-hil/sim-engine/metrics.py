"""Recovery metrics for one fault event. Pure scalars in, dict out; knows nothing about the link or the MCU.

Definitions (all measured from the fault instant t0, in ms unless noted):
- downtime:      t0 -> first frame of any kind from the MCU (it is alive again).
- time_to_ctrl:  t0 -> first frame with ctrl_active (it is flying the vehicle again).
                 Cold boot: downtime is short, time_to_ctrl includes the gyro calibration.
- max_tilt_deg:  worst ground-truth tilt from t0 until the window closes.
- settling:      t0 -> start of the first quiet period (tilt < SETTLE_TILT_DEG for SETTLE_HOLD_S)
                 after control resumed. None until settled.
- max_altitude_drop_m: worst loss of altitude below the altitude at t0.
- drift_deg:     worst angle between ground-truth attitude and the attitude the MCU reported,
                 over frames received since t0 (the stale-state / re-seed error).
"""
import config as C


class RecoveryMetrics:
    def __init__(self):
        self.active = False
        self.reset()

    def reset(self):
        self.t0 = None
        self.downtime = self.ttc = self.settling = None
        self.max_tilt = self.drift = self.max_drop = 0.0
        self.alt0 = None
        self._quiet_since = None
        self.settled = False

    def begin(self, t, alt):
        """Call when a fault is injected (restarts the measurement)."""
        self.reset()
        self.t0, self.alt0, self.active = t, alt, True

    def update(self, t, tilt_deg, got_frame, ctrl_active, alt, drift_deg=None):
        """Call every physics step. `drift_deg` is only given on steps where a frame arrived."""
        if not self.active:
            return
        self.max_tilt = max(self.max_tilt, tilt_deg)
        self.max_drop = max(self.max_drop, float(self.alt0 - alt))
        if got_frame:
            if self.downtime is None:
                self.downtime = (t - self.t0) * 1e3
            if drift_deg is not None:
                self.drift = max(self.drift, drift_deg)
        if ctrl_active and self.ttc is None:
            self.ttc = (t - self.t0) * 1e3
            self._quiet_since = t

        if self.ttc is not None:
            if tilt_deg > C.SETTLE_TILT_DEG:
                self._quiet_since = t
            elif t - self._quiet_since >= C.SETTLE_HOLD_S:
                self.settling = (self._quiet_since - self.t0) * 1e3
                self.settled, self.active = True, False
                return
        if t - self.t0 > C.METRICS_WINDOW_S:
            self.active = False                    # never settled; settling stays None

    def snapshot(self):
        r = lambda v: None if v is None else round(v, 1)
        return {"active": self.active, "settled": self.settled,
                "downtime_ms": r(self.downtime), "time_to_control_ms": r(self.ttc),
                "max_tilt_deg": round(self.max_tilt, 2), "settling_ms": r(self.settling),
                "drift_deg": round(self.drift, 2),
                "max_altitude_drop_m": round(self.max_drop, 2), "t0": self.t0}
