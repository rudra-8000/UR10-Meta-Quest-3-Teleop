#!/usr/bin/env python3
"""
Quest 3  ──WSS──▶  Python (HTTPS/aiohttp)  ──RTDE──▶  URSim / UR10

State machine:
  IDLE        ─ left GRIP edge   ──▶  CALIBRATED   (snapshot reference)
  CALIBRATED  ─ right TRIGGER>0.5 ──▶  TRACKING    (servoL loop)
  TRACKING    ─ right TRIGGER≤0.5 ──▶  CALIBRATED  (servoStop, hold position)
  CALIBRATED  ─ left GRIP edge   ──▶  IDLE          (full reset)
"""

import asyncio, json, ssl, socket, threading, time
from pathlib import Path

import aiohttp
from aiohttp import web
import numpy as np
from scipy.spatial.transform import Rotation as R
import rtde_control
import rtde_receive

# ═══════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

URSIM_IP     = "127.0.0.1"   # Docker maps UR ports → localhost
SERVER_PORT  = 8443           # HTTPS + WSS on one port
CERT_FILE    = "cert.pem"
KEY_FILE     = "key.pem"
HTML_FILE    = "index.html"

# ── Servo loop ──────────────────────────────────────────────────────
SERVO_DT         = 0.008   # 125 Hz  (must match actual loop period)
SERVO_LOOKAHEAD  = 0.08    # [s]  larger → smoother, more lag
SERVO_GAIN       = 300
MAX_VEL          = 0.25    # [m/s]
MAX_ACC          = 0.5     # [m/s²]

# ── Teleoperation scaling ───────────────────────────────────────────
POS_SCALE = 0.8  # robot metres per hand metre  (reduce for fine work)

# ── Workspace safety clamp (UR10 base frame, metres) ───────────────
TCP_MIN = np.array([-0.9, -0.9,  0.05])
TCP_MAX = np.array([ 0.9,  0.9,  1.10])

# ── Coordinate frame: Quest WebXR → UR10 base frame ────────────────
#
#  WebXR  (right-hand, Y-up):  X = right,   Y = up,  Z = towards user
#  UR10   (right-hand, Z-up):  X = forward, Y = left, Z = up
#
#  Default setup: user stands facing the robot front panel.
#
#    Quest  X (right)  →  Robot –Y  (robot's left when you face it)
#    Quest  Y (up)     →  Robot  Z  (up)
#    Quest  Z (back)   →  Robot  X  (forward into workspace)
#
#  Tune this matrix first by running in IDLE and printing delta_pos
#  to verify axes feel natural.
#
COORD_R = R.from_matrix(np.array([
    [ 0,  0,  -1],   # Robot X  = Quest  Z
    [-1,  0,  0],   # Robot Y  = –Quest X
    [ 0,  1,  0],   # Robot Z  = Quest  Y
]))

# ═══════════════════════════════════════════════════════════════════
#  Shared state  (WS handler writes, teleop thread reads)
# ═══════════════════════════════════════════════════════════════════

_lock = threading.Lock()
_ctrl = {"left": None, "right": None, "ts": 0.0}

# ═══════════════════════════════════════════════════════════════════
#  aiohttp handlers
# ═══════════════════════════════════════════════════════════════════

async def handle_index(request):
    return web.FileResponse(HTML_FILE)

async def handle_ws(request):
    ws = web.WebSocketResponse(heartbeat=10.0)
    await ws.prepare(request)
    print(f"[WS ] connected  {request.remote}")
    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            try:
                data = json.loads(msg.data)
                with _lock:
                    _ctrl["left"]  = data.get("left")
                    _ctrl["right"] = data.get("right")
                    _ctrl["ts"]    = time.monotonic()
            except Exception:
                pass
        elif msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
            break
    print("[WS ] disconnected")
    return ws

# ═══════════════════════════════════════════════════════════════════
#  Coordinate helpers
# ═══════════════════════════════════════════════════════════════════

def q2rp(pos: list) -> np.ndarray:
    """Quest position  →  UR10 base frame."""
    return COORD_R.apply(np.array(pos, dtype=float))

def q2rr(quat: list) -> R:
    """Quest quaternion [x,y,z,w]  →  UR10 base frame rotation."""
    qr = R.from_quat(quat)               # scipy: [x,y,z,w]
    return COORD_R * qr * COORD_R.inv()

def clamp_pose(pose: np.ndarray) -> np.ndarray:
    p = pose.copy()
    p[:3] = np.clip(p[:3], TCP_MIN, TCP_MAX)
    return p

# ═══════════════════════════════════════════════════════════════════
#  Teleoperation thread
# ═══════════════════════════════════════════════════════════════════

def teleop_thread():
    print(f"[UR ] Connecting to URSim @ {URSIM_IP} …")
    print("      (Make sure the robot is powered ON in the web UI at")
    print("       http://localhost:6080  before running this script)\n")

    ctrl = rtde_control.RTDEControlInterface(URSIM_IP)
    recv = rtde_receive.RTDEReceiveInterface(URSIM_IP)
    print(f"[UR ] Connected  TCP = {np.round(recv.getActualTCPPose(), 4)}\n")

    print("  Controls")
    print("  ─────────────────────────────────────────────────────────")
    print("  Left  GRIP  (press)   →  Calibrate reference pose + start")
    print("  Right TRIGGER (hold)  →  Robot tracks right controller")
    print("  Right TRIGGER (rel.)  →  Pause (robot holds position)")
    print("  Left  GRIP  (again)   →  Full reset to IDLE")
    print("  ─────────────────────────────────────────────────────────\n")

    def snapshot():
        with _lock:
            return dict(_ctrl)

    def fresh(d):
        return (time.monotonic() - d["ts"]) < 0.5

    mode            = "IDLE"
    calib           = {}
    lg_prev         = 0.0
    rt_prev         = 0.0

    while True:
        t0 = time.monotonic()
        d  = snapshot()

        if not fresh(d) or d["left"] is None or d["right"] is None:
            time.sleep(SERVO_DT)
            continue

        left  = d["left"]
        right = d["right"]
        lg    = float(left.get("grip",    0.0))
        rt    = float(right.get("trigger", 0.0))

        # Edge detection
        lg_edge    = lg > 0.5 and lg_prev <= 0.5
        rt_on      = rt > 0.5
        rt_off_edge = rt <= 0.5 and rt_prev > 0.5

        # ── Transitions ──────────────────────────────────────────

        if lg_edge:
            if mode == "IDLE":
                home = np.array(recv.getActualTCPPose())
                calib = {
                    "r_pos":  q2rp(right["pos"]),
                    "r_rot":  q2rr(right["quat"]),
                    "l_pos":  q2rp(left["pos"]),   # stored for optional rel. mode
                    "l_rot":  q2rr(left["quat"]),
                    "home":   home,
                }
                mode = "CALIBRATED"
                print(f"[UR ] Calibrated  home = {np.round(home, 3)}")

            elif mode in ("CALIBRATED", "TRACKING"):
                try:    ctrl.servoStop(a=4.0)
                except: pass
                mode  = "IDLE"
                calib = {}
                print("[UR ] → IDLE (reset)")

            time.sleep(0.35)   # debounce

        elif rt_off_edge and mode == "TRACKING":
            try:    ctrl.servoStop(a=4.0)
            except: pass
            mode = "CALIBRATED"
            print("[UR ] Tracking paused")

        elif rt_on and mode == "CALIBRATED":
            mode = "TRACKING"
            print("[UR ] Tracking started")

        # ── ServoL when TRACKING ──────────────────────────────────

        if mode == "TRACKING":
            r_pos_now = q2rp(right["pos"])
            r_rot_now = q2rr(right["quat"])

            # Position delta in robot world frame
            delta_p = (r_pos_now - calib["r_pos"]) * POS_SCALE

            # Rotation delta (right controller rotation relative to calibration)
            delta_r = r_rot_now * calib["r_rot"].inv()

            # Apply deltas to robot home pose
            target_pos = calib["home"][:3] + delta_p
            target_rot = (delta_r * R.from_rotvec(calib["home"][3:6])).as_rotvec()

            target = clamp_pose(np.concatenate([target_pos, target_rot]))

            try:
                ctrl.servoL(target.tolist(),
                            MAX_VEL, MAX_ACC,
                            SERVO_DT, SERVO_LOOKAHEAD, SERVO_GAIN)
            except Exception as e:
                print(f"[UR ] servoL error: {e}")
                mode = "CALIBRATED"

        lg_prev = lg
        rt_prev = rt
        time.sleep(max(0.0, SERVO_DT - (time.monotonic() - t0)))

# ═══════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════
def get_lan_ip() -> str:
    """Uses UDP routing to discover the outbound LAN IP — never returns 127.x.x.x."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("8.8.8.8", 80))   # no data sent; just asks kernel for route
            return s.getsockname()[0]
        except Exception:
            return "0.0.0.0"


def main():
    import socket
    #lan_ip = socket.gethostbyname(socket.gethostname())
    lan_ip = get_lan_ip()

    # Start teleop thread
    t = threading.Thread(target=teleop_thread, daemon=True)
    t.start()

    # Build aiohttp app (HTTP + WS on one SSL port)
    app = web.Application()
    app.router.add_get("/",   handle_index)
    app.router.add_get("/ws", handle_ws)

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(CERT_FILE, KEY_FILE)

    print(f"\n[SRV] Open on your Quest Meta Browser:")
    print(f"      https://{lan_ip}:{SERVER_PORT}/")
    print(f"      (tap 'Advanced → Proceed' to accept the self-signed cert)\n")

    web.run_app(app, port=SERVER_PORT, ssl_context=ssl_ctx, print=lambda *_: None)

if __name__ == "__main__":
    main()
