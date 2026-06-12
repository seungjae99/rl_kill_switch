#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import subprocess
import json

import rclpy
import utm
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String, Bool
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Trigger

from kill_switch.frequency_monitor import FrequencyMonitor

ORIGIN_LAT, ORIGIN_LON = 36.3961255, 127.401612
E0, N0, *_ = utm.from_latlon(ORIGIN_LAT, ORIGIN_LON)


def enu_to_tank(e: float, n: float, u: float) -> tuple:
    return (e - E0, -u, n - N0)


class KillSwitchNode(Node):
    """Kill Switch node: monitors system health and overrides PWM on failure.

    Subscribes to /tug1/PWM_raw from the Inference Node, relays it to
    /tug1/PWM when all triggers are clear, or outputs [0.0, 0.0] when any
    trigger is active.  Also owns the Arduino serial connection.
    """

    def __init__(self) -> None:
        super().__init__('tug1_kill_switch_node')

        # 1. Declare and load parameters
        self.declare_parameter('own_name', 'tug1')
        self.declare_parameter('origin_lat', 36.3961255)
        self.declare_parameter('origin_lon', 127.401612)
        self.declare_parameter('area_m', 33.0)
        # boundary stored as flat [lat1, lon1, lat2, lon2, ...]
        self.declare_parameter('boundary_gps_points', [
            36.3960255, 127.4014620,
            36.3960255, 127.4017620,
            36.3963255, 127.4017620,
            36.3963255, 127.4014620,
        ])
        self.declare_parameter('boundary_check_enabled', True)
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('rc_timeout_sec', 1.0)
        self.declare_parameter('main_pc_ip', '192.168.1.100')
        self.declare_parameter('ping_timeout_sec', 1.0)
        self.declare_parameter('ping_threshold_ms', 500.0)
        self.declare_parameter('ping_interval_sec', 1.0)
        self.declare_parameter('gps_min_hz', 3.0)
        self.declare_parameter('gps_consecutive_fail', 3)
        self.declare_parameter('imu_min_hz', 30.0)
        self.declare_parameter('imu_consecutive_fail', 3)
        self.declare_parameter('thruster_pwm_min', 1500.0)
        self.declare_parameter('thruster_pwm_max', 1730.0)
        self.declare_parameter('servo_pwm_min', 1100.0)
        self.declare_parameter('servo_pwm_max', 1900.0)
        self.declare_parameter('kill_pwm_value', 0.0)
        self.declare_parameter('auto_recovery_triggers', ['T3', 'T4', 'T6'])
        self.declare_parameter('manual_reset_triggers', ['T1', 'T2', 'T5'])
        self.declare_parameter('monitor_rate_hz', 50.0)

        own: str = self.get_parameter('own_name').value
        self._boundary_enabled: bool = self.get_parameter('boundary_check_enabled').value
        self._rc_timeout_sec: float = self.get_parameter('rc_timeout_sec').value
        self._main_pc_ip: str = self.get_parameter('main_pc_ip').value
        self._ping_threshold_ms: float = self.get_parameter('ping_threshold_ms').value
        self._ping_interval_sec: float = self.get_parameter('ping_interval_sec').value
        self._thr_min: float = self.get_parameter('thruster_pwm_min').value
        self._thr_max: float = self.get_parameter('thruster_pwm_max').value
        self._srv_min: float = self.get_parameter('servo_pwm_min').value
        self._srv_max: float = self.get_parameter('servo_pwm_max').value
        self._kill_pwm: float = self.get_parameter('kill_pwm_value').value
        monitor_rate_hz: float = self.get_parameter('monitor_rate_hz').value
        serial_port: str = self.get_parameter('serial_port').value
        serial_baud: int = self.get_parameter('serial_baud').value
        gps_min_hz: float = self.get_parameter('gps_min_hz').value
        gps_fail: int = self.get_parameter('gps_consecutive_fail').value
        imu_min_hz: float = self.get_parameter('imu_min_hz').value
        imu_fail: int = self.get_parameter('imu_consecutive_fail').value

        # 2. Internal state
        self._lock = threading.Lock()
        self._state: str = 'NORMAL'
        self._kill_active: bool = False
        self._triggers: dict = {f'T{i}': False for i in range(1, 7)}
        self._last_rc_time: float = time.time()
        self._manual_reset_triggers: list = self.get_parameter('manual_reset_triggers').value
        self._auto_recovery_triggers: list = self.get_parameter('auto_recovery_triggers').value

        # 3. FrequencyMonitor instances (GPS: 3 Hz, IMU: 30 Hz)
        self._gps_monitor = FrequencyMonitor(window_sec=1.0, min_hz=gps_min_hz, consecutive_fail=gps_fail)
        self._imu_monitor = FrequencyMonitor(window_sec=1.0, min_hz=imu_min_hz, consecutive_fail=imu_fail)

        # 4. Boundary polygon — convert GPS pairs to local tank coordinates
        flat: list = self.get_parameter('boundary_gps_points').value
        self._boundary_local: list = []
        for i in range(0, len(flat) - 1, 2):
            try:
                e, n, *_ = utm.from_latlon(flat[i], flat[i + 1])
                x, _, z = enu_to_tank(e, n, 0.0)
                self._boundary_local.append((x, z))
            except Exception as ex:
                self.get_logger().error(f'Boundary GPS conversion error at index {i}: {ex}')

        # 5. Serial initialization
        self._ser: serial.Serial | None = None
        try:
            self._ser = serial.Serial(serial_port, serial_baud, timeout=0.1)
            self.get_logger().info(f'Serial opened: {serial_port} @ {serial_baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial init error: {e}')

        # 6. Publishers
        self.pub_pwm = self.create_publisher(Float64MultiArray, f'/{own}/PWM', 10)
        self.pub_status = self.create_publisher(String, f'/{own}/kill_switch/status', 10)
        self.pub_triggered = self.create_publisher(Bool, f'/{own}/kill_switch/triggered', 10)

        # 7. Subscribers
        self.create_subscription(Float64MultiArray, f'/{own}/PWM_raw', self._cb_pwm_raw, 10)
        self.create_subscription(NavSatFix, f'/{own}/ublox_gps_node/fix', self._cb_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, f'/{own}/imu/data', self._cb_imu, qos_profile_sensor_data)

        # 8. Service
        self.create_service(Trigger, f'/{own}/kill_switch/reset', self._cb_reset)

        # 9. 50 Hz monitor timer
        self.create_timer(1.0 / monitor_rate_hz, self._timer_cb)

        # 10. Background threads
        threading.Thread(target=self._ping_thread_loop, daemon=True).start()
        threading.Thread(target=self._serial_reader_thread, daemon=True).start()

        self.get_logger().info('KillSwitchNode initialized.')

    # ─────────── ROS callbacks ───────────

    def _cb_pwm_raw(self, msg: Float64MultiArray) -> None:
        """T5 range check + relay or override + serial send."""
        with self._lock:
            thr, srv = msg.data[0], msg.data[1]

            pwm_invalid = (
                not (self._thr_min <= thr <= self._thr_max) or
                not (self._srv_min <= srv <= self._srv_max)
            )
            self._set_trigger('T5', pwm_invalid)

            out = Float64MultiArray()
            if self._kill_active:
                out.data = [self._kill_pwm, self._kill_pwm]
            else:
                out.data = list(msg.data)

            self.pub_pwm.publish(out)
            self._serial_send(out.data[0], out.data[1])

    def _cb_gps(self, msg: NavSatFix) -> None:
        """T1 boundary check + T4 GPS frequency monitor."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        try:
            e, n, *_ = utm.from_latlon(msg.latitude, msg.longitude)
            x, _, z = enu_to_tank(e, n, 0.0)
        except Exception as ex:
            self.get_logger().error(f'GPS conversion error: {ex}')
            return

        gps_triggered = self._gps_monitor.update()

        with self._lock:
            self._set_trigger('T4', gps_triggered)
            if self._boundary_enabled and self._boundary_local:
                outside = not self._point_in_polygon(x, z, self._boundary_local)
                self._set_trigger('T1', outside)

    def _cb_imu(self, msg: Imu) -> None:
        """T6 IMU frequency monitor."""
        imu_triggered = self._imu_monitor.update()
        with self._lock:
            self._set_trigger('T6', imu_triggered)

    def _cb_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Service handler: clear ARMED state if all manual triggers are resolved."""
        with self._lock:
            manual_still_active = any(
                self._triggers.get(t, False) for t in self._manual_reset_triggers
            )
            if manual_still_active:
                response.success = False
                response.message = 'Manual triggers still active. Resolve before reset.'
            else:
                self._state = 'NORMAL'
                self._kill_active = False
                response.success = True
                response.message = 'Reset successful.'
        return response

    def _timer_cb(self) -> None:
        """50 Hz: RC timeout check + status publish."""
        self._check_rc_timeout()
        self._publish_status()

    # ─────────── Core logic ───────────

    def _point_in_polygon(self, px: float, pz: float, polygon: list) -> bool:
        """Ray-casting algorithm. Returns True if (px, pz) is inside polygon."""
        inside = False
        n = len(polygon)
        j = n - 1
        for i in range(n):
            xi, zi = polygon[i]
            xj, zj = polygon[j]
            if ((zi > pz) != (zj > pz)) and \
               (px < (xj - xi) * (pz - zi) / (zj - zi) + xi):
                inside = not inside
            j = i
        return inside

    def _set_trigger(self, tid: str, active: bool) -> None:
        """Update trigger state and advance the state machine.

        Must be called with self._lock held.
        """
        prev = self._triggers.get(tid, False)
        self._triggers[tid] = active

        if active and not prev:
            self.get_logger().warn(f'[KillSwitch] {tid} TRIGGERED')

        any_active = any(self._triggers.values())

        if any_active:
            manual = any(self._triggers.get(t, False) for t in self._manual_reset_triggers)
            if manual:
                self._state = 'ARMED'
            else:
                self._state = 'TRIGGERED'
        else:
            if self._state != 'ARMED':
                self._state = 'NORMAL'

        self._kill_active = (self._state != 'NORMAL')

    def _publish_status(self) -> None:
        """Publish JSON state to /tug1/kill_switch/status and Bool to /tug1/kill_switch/triggered."""
        with self._lock:
            status = {
                'state': self._state,
                'kill_active': self._kill_active,
                'triggers': dict(self._triggers),
                'timestamp': time.time(),
            }

        str_msg = String()
        str_msg.data = json.dumps(status)
        self.pub_status.publish(str_msg)

        bool_msg = Bool()
        bool_msg.data = status['kill_active']
        self.pub_triggered.publish(bool_msg)

    # ─────────── Serial ───────────

    def _serial_send(self, thr: float, srv: float) -> None:
        """Send PWM command to Arduino. Must be called with self._lock held."""
        if self._ser is None:
            return
        try:
            cmd = f"T:{int(thr)},S:{int(srv)}\n"
            self._ser.write(cmd.encode())
            self._ser.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _serial_reader_thread(self) -> None:
        """Background thread: receive Arduino RC heartbeat (T2)."""
        while rclpy.ok():
            if self._ser is None:
                time.sleep(0.1)
                continue
            try:
                if self._ser.in_waiting:
                    line = self._ser.readline().decode().strip()
                    if line == 'RC:1':
                        with self._lock:
                            self._last_rc_time = time.time()
                            self._set_trigger('T2', False)
                    elif line == 'RC:0':
                        with self._lock:
                            self._set_trigger('T2', True)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
            time.sleep(0.02)

    def _check_rc_timeout(self) -> None:
        """Trigger T2 if last RC:1 exceeded rc_timeout_sec. Called from timer."""
        with self._lock:
            elapsed = time.time() - self._last_rc_time
            if elapsed > self._rc_timeout_sec:
                self._set_trigger('T2', True)

    # ─────────── Ping ───────────

    def _ping_thread_loop(self) -> None:
        """Background thread: 1 Hz ICMP ping to Main PC (T3)."""
        while rclpy.ok():
            rtt = self._ping_once(self._main_pc_ip)
            with self._lock:
                if rtt is None:
                    self._set_trigger('T3', True)
                else:
                    self._set_trigger('T3', rtt >= self._ping_threshold_ms)
            time.sleep(self._ping_interval_sec)

    def _ping_once(self, host: str) -> float | None:
        """Execute a single ICMP ping. Returns RTT in ms, or None on failure."""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', host],
                capture_output=True, text=True, timeout=2.0
            )
            if result.returncode == 0:
                for token in result.stdout.split():
                    if token.startswith('time='):
                        return float(token.split('=')[1])
        except Exception:
            pass
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KillSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
