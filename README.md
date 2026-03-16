# xarm_teleop

A ROS 2 Python package that implements a **master-slave teleoperation system** for dual xArm robots. The master arm's Cartesian position is read in real time and streamed over WebSocket, while the slave arm uses real-time Inverse Kinematics (IK) combined with PID or Computed-Torque Control (CTC) to track the master.

---

## Architecture

```
Master Arm (Physical) → master_ws.py  (WebSocket Server @ ws://127.0.0.1:8765)
                                ↓
                        slave_ik.py   (WebSocket Client + ROS 2 Node)
                                ↓
                  /servo_server/delta_twist_cmds  →  Slave Arm (Physical)
```

### Nodes

| Node | Entry point | Description |
|------|-------------|-------------|
| `master` | `xarm_teleop.master_ws:main` | Connects to the master xArm, reads end-effector position at 50 Hz, and broadcasts it as JSON over WebSocket. |
| `slave` | `xarm_teleop.slave_ik:main` | ROS 2 node that receives master position over WebSocket, solves real-time IK, and publishes `TwistStamped` commands to ROS Servo Server. |

---

## Features

- **50 Hz control loop** for low-latency teleoperation.
- **6-DOF Denavit-Hartenberg kinematics** — Forward Kinematics (FK) and numerical Jacobian.
- **Weighted damped least-squares IK** with null-space control for joint-limit avoidance.
- **Two control modes** selectable at runtime:
  - `pid` — PID with integral anti-windup.
  - `ctc` — Computed-Torque Control using a full dynamics model (mass matrix, Coriolis, gravity, friction).
- **TF2 integration** — reads actual end-effector pose from the TF tree when available, falls back to FK.
- **Graceful WebSocket handling** — reconnects on master disconnection.

---

## Requirements

### Hardware
- Master robot: xArm (reachable at `192.168.1.167` by default).
- Slave robot: xArm with a running ROS 2 driver and ROS Servo Server.

### Software
- ROS 2 (Humble or later)
- Python 3.8+
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)

### Python dependencies
```
numpy
websockets          # WebSocket server (master)
websocket-client    # WebSocket client (slave)
```

Install with:
```bash
pip install numpy websockets websocket-client
```

---

## Build

Clone this package inside your ROS 2 workspace and build it:

```bash
cd ~/ros2_ws/src
git clone <repo-url> xarm_teleop
cd ~/ros2_ws
colcon build --packages-select xarm_teleop
source install/setup.bash
```

---

## Usage

### 1. Start the Master node

Connect the master xArm to the network and run:

```bash
ros2 run xarm_teleop master
```

The node will:
1. Connect to the xArm at `192.168.1.167`.
2. Start a WebSocket server at `ws://127.0.0.1:8765`.
3. Stream `{"x": …, "y": …, "z": …}` (in meters) at 50 Hz.

### 2. Start the Slave node

In a second terminal (with the ROS 2 driver and Servo Server already running for the slave arm):

```bash
ros2 run xarm_teleop slave
```

The node will:
1. Subscribe to `/joint_states` for joint feedback.
2. Connect to the master WebSocket server.
3. Run the IK + controller loop at 50 Hz.
4. Publish `TwistStamped` commands to `/servo_server/delta_twist_cmds`.

---

## Configuration

Key parameters are defined at the top of each source file:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| Master IP | `master_ws.py` | `192.168.1.167` | IP address of the master xArm |
| WebSocket host/port | `master_ws.py` | `127.0.0.1:8765` | WebSocket server address |
| Control mode | `slave_ik.py` | `"pid"` | `"pid"` or `"ctc"` |
| Control rate | `slave_ik.py` | `50` Hz | Update frequency |
| Max twist | `slave_ik.py` | `±0.15 m/s` | Velocity saturation |
| IK damping λ | `slave_ik.py` | `0.015` | Singularity robustness |

To switch to Computed-Torque Control, change the `mode` attribute inside `SlaveControllerNode.__init__`:

```python
self.mode = "ctc"
```

---

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Subscribed | Slave arm joint positions and velocities |
| `/servo_server/delta_twist_cmds` | `geometry_msgs/TwistStamped` | Published | Cartesian velocity command for the servo server |

---

## Testing

```bash
colcon test --packages-select xarm_teleop
colcon test-result --verbose
```

Tests check PEP 8 style (`ament_flake8`), PEP 257 docstrings (`ament_pep257`), and copyright headers (`ament_copyright`).

---

## License

To be defined — see `package.xml`.
