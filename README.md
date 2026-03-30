# UR10 × Meta Quest 3 Teleoperation

Real-time teleoperation of a **Universal Robots UR10** arm using a **Meta Quest 3** headset as the 6-DoF input device. Hand-controller pose (position + orientation) is streamed from the Quest browser via WebXR → WebSocket → Python → RTDE, giving you direct Cartesian control of the robot's TCP at up to **125 Hz**.

---

## How It Works

```
Meta Quest 3 (WebXR)
      │
      │  WSS (WebSocket over TLS)
      ▼
Python Server  (aiohttp, HTTPS + WSS)
      │
      │  RTDE (Real-Time Data Exchange)
      ▼
  UR10 / URSim
```

1. `teleop_server.py` starts an **HTTPS + WebSocket server** on your LAN (port `8443`).
2. You open the served `index.html` on the **Quest Meta Browser** — WebXR grants access to controller poses each frame.
3. The browser sends a JSON payload (`{left: {...}, right: {...}}`) over a secure WebSocket at each animation frame.
4. The Python server decodes pose data, applies a **coordinate-frame transform** (WebXR Y-up → UR10 Z-up), and issues `servoL` commands via **RTDE** to track the right controller's pose relative to a calibrated reference.

---

## State Machine

The teleoperation loop is governed by a simple 3-state machine driven by controller inputs:

```
 IDLE
  │  Left GRIP (press)  →  snapshot calibration reference
  ▼
 CALIBRATED
  │  Right TRIGGER > 0.5  →  begin servoL tracking
  ▼
 TRACKING
  │  Right TRIGGER ≤ 0.5  →  pause, robot holds position
  ▼
 CALIBRATED
  │  Left GRIP (press again)  →  full reset
  ▼
 IDLE
```

| Input | Effect |
|---|---|
| Left GRIP (from IDLE) | Snapshot current robot & controller pose as calibration reference |
| Right TRIGGER hold | Robot tracks right controller (servoL active) |
| Right TRIGGER release | Robot stops and holds position (servoStop) |
| Left GRIP (from CALIBRATED/TRACKING) | Full reset back to IDLE |

---

## Prerequisites

### Hardware
- Universal Robots **UR10** (or URSim for simulation)
- **Meta Quest 3** headset
- Both devices on the **same LAN**

### Software
- Python 3.9+
- [`ur-rtde`](https://sdurobotics.gitlab.io/ur-rtde/) — UR Real-Time Data Exchange library
- `aiohttp` — async HTTP + WebSocket server
- `numpy`, `scipy` — pose math
- `openssl` — to generate a self-signed TLS certificate (required for WebXR)

Install Python dependencies:
```bash
pip install aiohttp numpy scipy ur-rtde
```

---

## Setup

### 1. Generate TLS Certificate

WebXR in the Quest browser **requires HTTPS**. Generate a self-signed certificate:

```bash
bash generate_cert.sh
```

This produces `cert.pem` and `key.pem` in the project root.

### 2. Configure the Server

Open `teleop_server.py` and set the IP address of your UR10 / URSim:

```python
URSIM_IP = "127.0.0.1"   # Change to your robot's IP, e.g. "192.168.1.100"
```

Optionally tune these parameters:

| Parameter | Default | Description |
|---|---|---|
| `SERVO_DT` | `0.008` s | Servo loop period (125 Hz) |
| `SERVO_LOOKAHEAD` | `0.08` s | ServoL lookahead — larger = smoother but laggier |
| `SERVO_GAIN` | `300` | ServoL gain |
| `POS_SCALE` | `0.8` | Robot metres per hand metre (reduce for precision tasks) |
| `MAX_VEL` | `0.25` m/s | Maximum TCP velocity |
| `MAX_ACC` | `0.5` m/s² | Maximum TCP acceleration |
| `TCP_MIN/MAX` | see code | Workspace safety clamp in UR10 base frame |

### 3. Start the Server

```bash
python teleop_server.py
```

You should see:
```
[SRV] Open on your Quest Meta Browser:
      https://<your-lan-ip>:8443/
      (tap 'Advanced → Proceed' to accept the self-signed cert)
```

### 4. Connect the Quest

1. Put on the Meta Quest 3.
2. Open the **Meta Browser** and navigate to `https://<your-lan-ip>:8443/`.
3. Accept the self-signed certificate warning (tap **Advanced → Proceed**).
4. Grant **controller tracking** permission when prompted.
5. The WebXR session will start — controller poses are now streaming to the server.

---

## Coordinate Frame Mapping

WebXR uses a right-handed, **Y-up** frame. The UR10 base frame is right-handed, **Z-up**. The following mapping is applied (assuming the user stands facing the robot's front panel):

| Quest Axis | UR10 Axis | Meaning |
|---|---|---|
| +X (right) | −Y | Robot's left |
| +Y (up) | +Z | Vertical |
| +Z (toward user) | +X | Forward into workspace |

This is encoded as a fixed rotation matrix `COORD_R` in `teleop_server.py`. If your setup orientation differs, tune this matrix first by running in `IDLE` mode and verifying that hand movements map to the expected robot directions.

---

## File Structure

```
.
├── teleop_server.py      # Main server: aiohttp + RTDE teleop thread
├── teleop_server_2.py    # Alternate version (development)
├── teleop_server_3.py    # Alternate version (development)
├── index.html            # WebXR frontend served to the Quest browser
├── index_2.html          # Alternate frontend (development)
├── index_3.html          # Alternate frontend (development)
├── cert.pem              # TLS certificate (generated)
├── key.pem               # TLS private key (generated)
└── generate_cert.sh      # Script to generate self-signed cert
```

---

## Safety Notes

- Always ensure the **workspace safety clamp** (`TCP_MIN` / `TCP_MAX`) is configured correctly for your physical setup before running on real hardware.
- The `POS_SCALE` parameter scales hand motion to robot motion — start with a **low value** (e.g. `0.3`) when testing for the first time.
- Releasing the **right trigger** immediately issues `servoStop` — keep your finger near the trigger at all times to pause motion.
- Test with **URSim** (Docker) before running on a physical robot.

### Running URSim (Docker)

```bash
docker run --rm -it -p 5900:5900 -p 6080:6080 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series
```

Open `http://localhost:6080` in a browser, power on the robot, then start `teleop_server.py`.

---

## Acknowledgements

- [UR RTDE Python Client](https://sdurobotics.gitlab.io/ur-rtde/) — Universal Robots Real-Time Data Exchange
- [WebXR Device API](https://www.w3.org/TR/webxr/) — W3C spec enabling VR/AR controller access in browsers
- [aiohttp](https://docs.aiohttp.org/) — async Python HTTP server
