"""Shared constants. SI units, NED world frame, FRD body frame (x fwd, y right, z down)."""
import numpy as np

# --- timing -----------------------------------------------------------------
DT = 0.005                 # physics + control period [s] (200 Hz)
WS_HZ = 60                 # dashboard broadcast rate
BAUD = 921_600             # UART to the real STM32 (SerialDevice only)

# --- airframe (X-quad) -------------------------------------------------------
G = 9.80665                                  # matches firmware README v1 example
MASS = 1.0                                   # [kg]
J = np.diag([0.010, 0.010, 0.018])           # inertia [kg m^2]
ARM = 0.12                                   # motor offset along body x and y [m]
KQ = 0.016                                   # yaw torque per newton of thrust [m]
T_MAX = 6.0                                  # max thrust per motor [N]
ESC_TAU = 0.04                               # ESC+motor first-order lag [s]
DRAG = 0.25                                  # linear drag [N s/m]; also what lets the accel "see" tilt
# Motor order FR, RL, FL, RR (PX4 quad-X). Positions in FRD, spin sign: +1 = CCW prop (reaction torque +z).
MOTOR_X = ARM * np.array([+1, -1, +1, -1])
MOTOR_Y = ARM * np.array([+1, -1, -1, +1])
MOTOR_SPIN = np.array([+1, +1, -1, -1])
HOVER_THROTTLE = MASS * G / (4 * T_MAX)

# --- disturbance + sensors ---------------------------------------------------
# A gust the held ESC command cannot cancel: this is what makes an MCU outage physically costly.
DIST_TORQUE = np.array([0.004, 0.002, 0.0])  # constant body torque [N m]
GUST_TORQUE = np.array([0.012, 0.008, 0.0])    # sinusoidal body torque amplitude [N m]
GUST_HZ = 0.7
GYRO_BIAS = np.array([0.020, -0.015, 0.010])  # true gyro bias [rad/s] (what cold boot must re-learn)
GYRO_NOISE = 0.003                            # [rad/s]
ACCEL_NOISE = 0.05                            # [m/s^2]
MAG_FIELD_NED = np.array([22.0, 0.0, 44.0])   # world magnetic field [uT], north + down (firmware README v1)
MAG_NOISE = 0.3                               # [uT]
START_ALT = 30.0                              # initial altitude [m]; ground is hard clamp at 0

# --- mock MCU: fault + boot timing [s] ---------------------------------------
IWDG_TIMEOUT = 0.100       # hung task until IWDG bites
WWDG_WINDOW = 0.010        # late task until WWDG bites
TORN_WRITE_DELAY = 0.005   # reset lands mid state-write
BOOT_COLD = 0.150          # full peripheral init, then...
GYRO_CAL = 2.5             # ...gyro-bias calibration (MCU alive, not controlling)
BOOT_WARM = 0.012          # critical-only init incl. USART+DMA re-init
REALIGN = 0.150            # warm fallback: attitude re-seeded from accel+mag before control resumes
# Warm recovery rule (firmware README "Watchdog warm start"): accept the newest intact checkpoint only if
CHECKPOINT_MAX_AGE = 0.250    # ...it is younger than this (fresh sample time - checkpoint sample time) [s]
ATTITUDE_REJECT_DEG = 10.0    # ...and its tilt agrees with the fresh accelerometer within this
BIAS_LIMIT = 0.1              # saved gyro bias reused only if every axis is below this [rad/s]
PERSIST_INTEGRATOR = False  # README TBD: is the PID integrator part of the saved state?

# --- mock MCU: estimator + controller ----------------------------------------
MAHONY_KP = 1.5
MAHONY_KI = 0.02
REALIGN_KP = 8.0           # aggressive accel correction while re-seeding
ANGLE_KP = np.array([6.0, 6.0, 2.0])   # angle error -> rate setpoint [1/s]
RATE_KP = np.array([15.0, 15.0, 8.0])  # rate error -> angular accel [1/s]
RATE_KI = np.array([20.0, 20.0, 4.0])
INT_LIMIT = 2.0

# --- metrics -----------------------------------------------------------------
SETTLE_TILT_DEG = 2.0      # "settled" = tilt below this...
SETTLE_HOLD_S = 0.5        # ...for this long, after control resumed
METRICS_WINDOW_S = 12.0    # give up waiting for settling after this long
