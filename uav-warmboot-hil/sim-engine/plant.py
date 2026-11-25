"""Pure 6-DOF quadcopter physics. No I/O, no MCU knowledge.

Conventions
- World NED (x north, y east, z DOWN), so altitude is -pos[2] and gravity is +z.
- Body FRD (x forward, y right, z down). Quaternion q = [w, x, y, z] rotates body -> NED.
- Thrust acts along body -z. A motor at body (x, y) with thrust T gives torque r x F:
  roll = -sum(y*T), pitch = sum(x*T), yaw = KQ * sum(spin*T).
- Motor commands are normalised 0..1 and pass through a first-order ESC lag.
"""
import numpy as np
import config as C


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


def qrot(q):
    """Rotation matrix body -> NED."""
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                     [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


def euler_deg(q):
    """Roll, pitch, yaw [deg] (ZYX), for display only."""
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.degrees([roll, pitch, yaw])


def tilt_deg(q):
    """Angle between body z and world down (ignores yaw)."""
    return float(np.degrees(np.arccos(np.clip(qrot(q)[2, 2], -1, 1))))


def angle_between_deg(q1, q2):
    """Smallest rotation angle between two attitudes."""
    return float(np.degrees(2 * np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1))))


class Plant:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.pos = np.array([0.0, 0.0, -C.START_ALT])
        self.vel = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)                       # body rates [rad/s]
        self.u = np.full(4, C.HOVER_THROTTLE)      # actual (post-ESC) motor throttle
        self._f_body = np.array([0.0, 0.0, -C.MASS * C.G])
        self._Jinv = np.linalg.inv(C.J)

    def step(self, cmd, dt=C.DT):
        """Advance one fixed step with normalised motor commands `cmd` (4,)."""
        self.u += (np.clip(cmd, 0, 1) - self.u) * (1 - np.exp(-dt / C.ESC_TAU))
        thrust = C.T_MAX * self.u
        R = qrot(self.q)

        f = np.array([0.0, 0.0, -thrust.sum()]) - C.DRAG * (R.T @ self.vel)
        gust = C.DIST_TORQUE + C.GUST_TORQUE * np.sin(2*np.pi*C.GUST_HZ*self.t)
        tau = np.array([-(C.MOTOR_Y * thrust).sum(),
                        (C.MOTOR_X * thrust).sum(),
                        C.KQ * (C.MOTOR_SPIN * thrust).sum()]) + gust

        # Semi-implicit Euler: rates first, then attitude via exponential map.
        self.w += self._Jinv @ (tau - np.cross(self.w, C.J @ self.w)) * dt
        ang = np.linalg.norm(self.w) * dt
        axis = self.w / (np.linalg.norm(self.w) + 1e-12)
        dq = np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * axis])
        self.q = qmul(self.q, dq)
        self.q /= np.linalg.norm(self.q)

        self.vel += (R @ f / C.MASS + np.array([0.0, 0.0, C.G])) * dt
        self.pos += self.vel * dt
        if self.pos[2] > 0:                        # hard ground: stop dead
            self.pos[2], self.vel[:] = 0.0, 0.0
        self._f_body = f
        self.t += dt

    def imu(self):
        """Sensor model, body frame: gyro [rad/s], accel specific force [m/s^2], mag [uT]."""
        gyro = self.w + C.GYRO_BIAS + self.rng.normal(0, C.GYRO_NOISE, 3)
        accel = self._f_body / C.MASS + self.rng.normal(0, C.ACCEL_NOISE, 3)
        mag = qrot(self.q).T @ C.MAG_FIELD_NED + self.rng.normal(0, C.MAG_NOISE, 3)
        return gyro, accel, mag
