"""Reference sensor log for offline firmware testing (parser, estimator, bias calibration).

Kinematic, not dynamic: attitude and acceleration follow a scripted profile so every truth value is exact.
Sensor readings use the same model as plant.imu(), plus optional per-run errors.

Profile (seq 0 at t = 0, one row per 5 ms):
     0 -  5 s  stationary      level, heading north
     5 -  7 s  yaw             +45 deg/s to heading 90 (east)
     7 -  8 s  hold
     8 -  9 s  roll            +20 deg/s to roll +20 deg
     9 - 10 s  hold
    10 - 11 s  roll back       -20 deg/s to level
    11 - 12 s  hold
    12 - 17 s  accelerate      +2.0 m/s^2 north, level, still facing east (shows on body -Y)
    17 - 19 s  cruise          10 m/s north, constant velocity

Conventions
    world NED, body FRD; q = [w,x,y,z] rotates body -> NED; Euler = ZYX (yaw, pitch, roll) in degrees
    true_rate = body angular rate [rad/s]; true_acc_ned = translational acceleration in NED [m/s^2], gravity excluded
    accel = specific force in body = R^T (true_acc_ned - [0,0,g]); reads [0,0,-g] when still and level

    python sensor_log.py                                   # defaults below, seed 0
    python sensor_log.py --gyro-bias 0.02 -0.015 0.01 --gyro-rw 0.0005 --seed 3 --out logs/run3.csv
"""
import argparse, csv
from pathlib import Path

import numpy as np
import config as C
from plant import qrot

SEGMENTS = [  # (end time [s], label, yaw rate [deg/s], roll rate [deg/s], north accel [m/s^2])
    (5, "stationary", 0, 0, 0), (7, "yaw", 45, 0, 0), (8, "hold", 0, 0, 0),
    (9, "roll", 0, 20, 0), (10, "hold", 0, 0, 0), (11, "roll_back", 0, -20, 0),
    (12, "hold", 0, 0, 0), (17, "accelerate", 0, 0, 2.0), (19, "cruise", 0, 0, 0)]


def quat_zyx(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (roll, pitch, yaw) for f in (np.cos, np.sin))
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gyro-bias", type=float, nargs=3, default=C.GYRO_BIAS.tolist(), help="rad/s")
    ap.add_argument("--gyro-rw", type=float, default=0.0, help="bias random walk, rad/s/sqrt(s)")
    ap.add_argument("--gyro-noise", type=float, default=C.GYRO_NOISE, help="rad/s, 1 sigma")
    ap.add_argument("--accel-bias", type=float, nargs=3, default=[0, 0, 0], help="m/s^2")
    ap.add_argument("--accel-noise", type=float, default=C.ACCEL_NOISE)
    ap.add_argument("--mag-bias", type=float, nargs=3, default=[0, 0, 0], help="uT (hard iron)")
    ap.add_argument("--mag-noise", type=float, default=C.MAG_NOISE)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(Path(__file__).parent / "logs" / "sensor_reference_log.csv"))
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    bias = np.array(a.gyro_bias, float)
    roll = yaw = 0.0
    vel_n = 0.0
    n_steps = int(round(SEGMENTS[-1][0] / C.DT))
    Path(a.out).parent.mkdir(exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_us", "seq", "segment",
                    "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z", "mag_x", "mag_y", "mag_z",
                    "q_w", "q_x", "q_y", "q_z", "roll_deg", "pitch_deg", "yaw_deg",
                    "true_rate_x", "true_rate_y", "true_rate_z", "true_acc_n", "true_acc_e", "true_acc_d",
                    "true_vel_n", "gyro_bias_x", "gyro_bias_y", "gyro_bias_z"])
        for k in range(n_steps + 1):
            t = k * C.DT
            label, yaw_rate, roll_rate, acc_n = next((s[1:] for s in SEGMENTS if t < s[0]), SEGMENTS[-1][1:])
            psi_dot, phi_dot = np.radians(yaw_rate), np.radians(roll_rate)
            # ZYX kinematics with pitch = 0: body rate from Euler rates
            rate = np.array([phi_dot, psi_dot * np.sin(roll), psi_dot * np.cos(roll)])
            q = quat_zyx(roll, 0.0, yaw)
            acc = np.array([acc_n, 0.0, 0.0])
            R = qrot(q)

            gyro = rate + bias + rng.normal(0, a.gyro_noise, 3)
            accel = R.T @ (acc - np.array([0, 0, C.G])) + a.accel_bias + rng.normal(0, a.accel_noise, 3)
            mag = R.T @ C.MAG_FIELD_NED + a.mag_bias + rng.normal(0, a.mag_noise, 3)
            w.writerow([int(round(t * 1e6)), k, label, *np.round(gyro, 6), *np.round(accel, 5), *np.round(mag, 4),
                        *(np.round(q, 7) + 0.0), round(np.degrees(roll), 4) + 0.0, 0.0, round(np.degrees(yaw), 4) + 0.0,
                        *(np.round(rate, 6) + 0.0), *(np.round(acc, 4) + 0.0), round(vel_n, 4), *np.round(bias, 6)])

            # advance truth to the next sample (exact for piecewise-constant rates)
            roll += phi_dot * C.DT
            yaw += psi_dot * C.DT
            vel_n += acc_n * C.DT
            bias += rng.normal(0, a.gyro_rw * np.sqrt(C.DT), 3)
    print(f"wrote {n_steps + 1} rows to {a.out}")


if __name__ == "__main__":
    main()
