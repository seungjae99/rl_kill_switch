# kill_switch

ROS2 safety node for the autonomous tugboat (`tug1`). Sits between the RL Inference Node and the Arduino: relays PWM commands when all safety conditions are met, overrides to `[0.0, 0.0]` (full stop) when any trigger fires, and owns the Arduino serial connection exclusively.

---

## System Overview

```
Inference Node ──/tug1/PWM_raw──▶ Kill Switch ──/tug1/PWM──▶ Arduino
                                        ▲
                             GPS · IMU · Serial · Ping
```

---

## Report

[View System Overview Report](https://seungjae99.github.io/rl_kill_switch/)

---

## Package Structure

```
kill_switch/
├── kill_switch/
│   ├── __init__.py
│   ├── kill_switch_node.py      # KillSwitchNode — main node
│   └── frequency_monitor.py    # FrequencyMonitor — sliding-window Hz checker
├── config/
│   └── kill_switch_params.yaml  # all tunable parameters
├── launch/
│   └── kill_switch.launch.py
├── resource/kill_switch
├── package.xml
├── setup.py
└── README.md
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select kill_switch
source install/setup.bash
```

---

## Run

```bash
ros2 launch kill_switch kill_switch.launch.py
```

Optional launch arguments:

```bash
ros2 launch kill_switch kill_switch.launch.py log_level:=debug
ros2 launch kill_switch kill_switch.launch.py namespace:=robot1
```

Or run the node directly:

```bash
ros2 run kill_switch kill_switch_node
```

---

## Topic Map

### Subscribed

| Topic | Type | Purpose |
|-------|------|---------|
| `/tug1/PWM_raw` | `Float64MultiArray` | Inference Node output — relay or override |
| `/tug1/ublox_gps_node/fix` | `NavSatFix` | T1 boundary check + T4 GPS frequency |
| `/tug1/imu/data` | `Imu` | T6 IMU frequency |

### Published

| Topic | Type | Content |
|-------|------|---------|
| `/tug1/PWM` | `Float64MultiArray` | `[thr, srv]` relay or `[0.0, 0.0]` on kill |
| `/tug1/kill_switch/status` | `String` | JSON: `{state, kill_active, triggers, timestamp}` |
| `/tug1/kill_switch/triggered` | `Bool` | `true` when kill is active |

### Service

| Service | Type | Purpose |
|---------|------|---------|
| `/tug1/kill_switch/reset` | `std_srvs/Trigger` | Clear `ARMED` state after resolving manual triggers |

---

## Triggers

| ID | Name | Detection method | Recovery |
|----|------|-----------------|----------|
| T1 | Boundary violation | GPS callback → Ray-casting against boundary polygon | Manual |
| T2 | RC link loss | Serial `RC:0` from Arduino, or no `RC:1` for `rc_timeout_sec` | Manual |
| T3 | Remote comms loss | ICMP ping to `main_pc_ip` — no reply or RTT >= threshold | Auto |
| T4 | GPS degraded | `FrequencyMonitor` — < 3 Hz for 3 consecutive windows | Auto |
| T5 | PWM out of range | `/tug1/PWM_raw` outside `[1500,1730]` / `[1100,1900]` bounds | Manual |
| T6 | IMU degraded | `FrequencyMonitor` — < 30 Hz for 3 consecutive windows | Auto |

**Manual triggers** (T1, T2, T5): require a `reset` service call to clear — state stays `ARMED` even after the cause disappears.

**Auto-recovery triggers** (T3, T4, T6): cleared automatically once the condition resolves — state returns to `NORMAL`.

---

## State Machine

```
NORMAL ──[any trigger fires]──▶ TRIGGERED  (auto-recovery triggers only)
                                     │
                              [T1/T2/T5 fires]
                                     ▼
                                  ARMED ──[/kill_switch/reset service]──▶ NORMAL
```

Both `TRIGGERED` and `ARMED` set `kill_active = True` and output `[0.0, 0.0]` to Arduino.

---

## Manual Reset

When T1, T2, or T5 fires the node locks into `ARMED`. After resolving the root cause:

```bash
ros2 service call /tug1/kill_switch/reset std_srvs/srv/Trigger {}
```

Reset is rejected while any manual-reset trigger is still active.

---

## Required Parameter Updates Before Deployment

Edit `config/kill_switch_params.yaml`:

| Parameter | Default | What to change |
|-----------|---------|----------------|
| `main_pc_ip` | `192.168.1.100` | Actual IP of the operator PC |
| `boundary_gps_points` | placeholder coords | Real GPS survey of tank boundary — flat list `[lat1, lon1, lat2, lon2, ...]` |
| `serial_port` | `/dev/ttyACM1` | Actual Arduino port (`ls /dev/ttyACM*`) |

> `boundary_gps_points` uses a flat 1-D array because ROS2 parameters do not support nested lists. Each consecutive pair `[lat_n, lon_n]` defines one polygon vertex.

---

## Monitor Status

```bash
# Watch JSON state (state, kill_active, per-trigger flags, timestamp)
ros2 topic echo /tug1/kill_switch/status

# Watch kill active flag
ros2 topic echo /tug1/kill_switch/triggered

# Confirm PWM is being forwarded correctly
ros2 topic echo /tug1/PWM
```

---

## Trigger Test Procedures

| Trigger | How to test | Expected result |
|---------|-------------|-----------------|
| T1 boundary | Replay rosbag with out-of-bounds GPS coordinates | `/tug1/PWM` → `[0.0, 0.0]`, Serial `T:0,S:0` |
| T2 RC loss | Disable Arduino RC channel and wait `rc_timeout_sec` (1 s) | `/tug1/PWM` → `[0.0, 0.0]` |
| T2 serial loss | Unplug `/dev/ttyACM1` | `/tug1/PWM` → `[0.0, 0.0]` |
| T3 comms loss | `sudo iptables -A OUTPUT -d <main_pc_ip> -j DROP` | `/tug1/PWM` → `[0.0, 0.0]` |
| T3 auto-recovery | Remove iptables rule | Relay resumes automatically |
| T4 GPS degraded | Publish `/tug1/ublox_gps_node/fix` at < 3 Hz for 3+ s | `/tug1/PWM` → `[0.0, 0.0]` |
| T5 PWM range | `ros2 topic pub /tug1/PWM_raw std_msgs/msg/Float64MultiArray "data: [950.0, 1000.0]"` | `/tug1/PWM` → `[0.0, 0.0]` |
| T6 IMU degraded | Publish `/tug1/imu/data` at < 30 Hz for 3+ s | `/tug1/PWM` → `[0.0, 0.0]` |
| Normal relay | `ros2 topic pub /tug1/PWM_raw std_msgs/msg/Float64MultiArray "data: [1600.0, 1500.0]"` | `/tug1/PWM` echoes same values |
