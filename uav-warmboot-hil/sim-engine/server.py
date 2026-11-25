"""Wall-clock 200 Hz physics loop (own thread) + 60 Hz FastAPI WebSocket broadcaster + CSV logger.

    python server.py                 # in-process MockMCU
    $env:HIL_PORT="COM5"; python server.py   # real STM32 over UART (hil_bridge.SerialDevice)

Dashboard -> server (WebSocket text): {"cmd": "iwdg_hang"|"wwdg_overrun"|"torn_write"|"cold_boot", "outage_ms": 0}
Server -> dashboard: one JSON snapshot per tick, see Sim.snapshot().
"""
import asyncio, csv, json, os, queue, sys, threading, time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

import config as C
from hil_bridge import HilBridge, SerialDevice, Fault, BootPath
from metrics import RecoveryMetrics
from mock_mcu import MockMCU
from plant import Plant, euler_deg, tilt_deg, angle_between_deg

FAULTS = {"iwdg_hang": Fault.IWDG_HANG, "wwdg_overrun": Fault.WWDG_OVERRUN,
          "torn_write": Fault.TORN_WRITE, "cold_boot": Fault.COLD_BOOT}
ONLINE_TIMEOUT = 4 * C.DT          # no frame for this long => MCU considered down
MAX_CATCHUP = 20                   # steps to replay after a stall before we give up and resync
LOG_DIR = Path(__file__).parent / "logs"


class Sim:
    def __init__(self):
        port = os.environ.get("HIL_PORT")
        self.bridge = HilBridge(SerialDevice(port) if port else MockMCU())
        self.plant, self.metrics = Plant(), RecoveryMetrics()
        self.motors = np.full(4, C.HOVER_THROTTLE)    # ESC input; held while no valid control frame
        self.est_quat, self.boot_path = np.array([1.0, 0, 0, 0]), BootPath.NONE
        self.last_frame_t, self.ctrl_active, self.overruns = 0.0, True, 0
        self.cmds, self.stop = queue.Queue(), threading.Event()
        self.snap = {}
        LOG_DIR.mkdir(exist_ok=True)
        self.log_file = open(LOG_DIR / f"flight_{datetime.now():%Y%m%d_%H%M%S}.csv", "w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(["t", "n", "e", "d", "roll", "pitch", "yaw", "est_roll", "est_pitch", "est_yaw",
                           "m1", "m2", "m3", "m4", "mcu_online", "ctrl_active", "boot_path", "tilt_deg", "drift_deg"])

    def command(self, msg):
        """Thread-safe: called from the WebSocket handler."""
        if msg.get("cmd") in FAULTS:
            self.cmds.put((FAULTS[msg["cmd"]], int(msg.get("outage_ms", 0))))

    def step(self):
        p = self.plant
        while not self.cmds.empty():
            fault, extra_ms = self.cmds.get()
            self.bridge.send_fault(fault, extra_ms)
            self.metrics.begin(p.t, -p.pos[2])

        p.step(self.motors)
        self.bridge.send_sensors(p.t, *p.imu())
        frames = self.bridge.poll(p.t)

        drift = None
        for f in frames:
            self.last_frame_t, self.boot_path = p.t, f.boot_path
            self.est_quat, self.ctrl_active = f.est_quat, f.ctrl_active
            if f.ctrl_active:
                self.motors = f.motors                # otherwise ESC holds the last valid command
            drift = angle_between_deg(p.q, f.est_quat)

        tilt = tilt_deg(p.q)
        self.metrics.update(p.t, tilt, bool(frames), bool(frames) and self.ctrl_active, -p.pos[2], drift)
        self.snap = self.snapshot(tilt, drift)

    def snapshot(self, tilt, drift):
        p = self.plant
        online = p.t - self.last_frame_t < ONLINE_TIMEOUT
        r, e = euler_deg(p.q), euler_deg(self.est_quat)
        self.log.writerow([f"{p.t:.4f}", *np.round(p.pos, 3), *np.round(r, 3), *np.round(e, 3),
                           *np.round(self.motors, 4), int(online), int(self.ctrl_active),
                           self.boot_path.name, round(tilt, 3), "" if drift is None else round(drift, 3)])
        if round(p.t / C.DT) % 200 == 0:
            self.log_file.flush()
        return {"t": round(p.t, 3), "pos": np.round(p.pos, 3).tolist(), "altitude_m": round(-p.pos[2], 2), "quat": np.round(p.q, 5).tolist(),
                "euler": np.round(r, 2).tolist(), "est_euler": np.round(e, 2).tolist(),
                "motors": np.round(self.motors, 3).tolist(),
                "mcu": {"online": bool(online), "ctrl_active": bool(self.ctrl_active),
                        "boot_path": self.boot_path.name},
                "metrics": self.metrics.snapshot(), "overruns": self.overruns}

    def run(self):
        """Fixed-step sim locked to the wall clock. Pure spin on perf_counter: no sleep jitter."""
        sys.setswitchinterval(5e-4)                   # let the asyncio thread in between spins
        t0, done = time.perf_counter(), 0
        while not self.stop.is_set():
            due = int((time.perf_counter() - t0) / C.DT)
            if due - done > MAX_CATCHUP:              # stalled: skip ahead rather than spiral
                self.overruns, done = self.overruns + 1, due - MAX_CATCHUP
            while done < due:
                self.step()
                done += 1
        self.log_file.close()


sim = Sim()
clients = set()


async def broadcaster():
    """60 Hz snapshot fan-out. Deadline-based so it doesn't drift; precision isn't critical here."""
    nxt = time.perf_counter()
    while True:
        nxt += 1 / C.WS_HZ
        if sim.snap and clients:
            text = json.dumps(sim.snap)
            for ws in list(clients):
                try:
                    await ws.send_text(text)
                except Exception:
                    clients.discard(ws)
        await asyncio.sleep(max(0.0, nxt - time.perf_counter()))


@asynccontextmanager
async def lifespan(app):
    threading.Thread(target=sim.run, daemon=True).start()
    task = asyncio.create_task(broadcaster())
    yield
    task.cancel()
    sim.stop.set()


app = FastAPI(lifespan=lifespan)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            sim.command(json.loads(await ws.receive_text()))
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="warning")
