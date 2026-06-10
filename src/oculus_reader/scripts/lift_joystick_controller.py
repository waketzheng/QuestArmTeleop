#!/usr/bin/env python3
import socket
import struct
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Int8, Int32, UInt16


OD_ERROR_CODE = (0x603F, 0x00)
OD_CONTROL_WORD = (0x6040, 0x00)
OD_STATUS_WORD = (0x6041, 0x00)
OD_MODE = (0x6060, 0x00)
OD_MODE_DISPLAY = (0x6061, 0x00)
OD_POS_ACTUAL = (0x6064, 0x00)
OD_VEL_ACTUAL = (0x606C, 0x00)
OD_TARGET_VEL = (0x60FF, 0x00)

MODE_PV = 3


class SimpleCanopenLift:
    def __init__(
        self,
        logger,
        can_interface: str,
        node_id: int,
        bitrate: int,
        velocity_command: int,
        sdo_timeout_s: float,
        pulse_per_mm: float,
        max_height: float,
        height_offset: float,
        raw_height_min: float,
        raw_height_max: float,
    ):
        self.logger = logger
        self.can_interface = can_interface
        self.node_id = int(node_id)
        self.bitrate = int(bitrate)
        self.velocity_command = int(velocity_command)
        self.sdo_timeout_s = float(sdo_timeout_s)
        self.pulse_per_mm = float(pulse_per_mm)
        self.max_height = float(max_height)
        self.height_offset = float(height_offset)
        self.raw_height_min = float(raw_height_min)
        self.raw_height_max = float(raw_height_max)
        self.socket: Optional[socket.socket] = None

    def connect(self):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.settimeout(self.sdo_timeout_s)
        self.socket.bind((self.can_interface,))
        self._send_nmt(0x01)
        time.sleep(0.1)
        self.logger.info(
            f"Lift CANopen connected: interface={self.can_interface}, "
            f"node_id={self.node_id}, bitrate={self.bitrate}"
        )

    def close(self):
        if self.socket is None:
            return
        try:
            self.control_stop()
        except Exception as exc:
            self.logger.warning(f"Failed to stop lift before CAN close: {exc}")
        self.socket.close()
        self.socket = None

    def control_rise(self):
        self._prepare_pv_mode()
        self._write_i32(OD_TARGET_VEL, abs(self.velocity_command))
        self.logger.info(
            f"Lift CAN command: rise target_velocity={abs(self.velocity_command)}"
        )

    def control_decline(self):
        self._prepare_pv_mode()
        self._write_i32(OD_TARGET_VEL, -abs(self.velocity_command))
        self.logger.info(
            f"Lift CAN command: decline target_velocity={-abs(self.velocity_command)}"
        )

    def control_stop(self):
        self._prepare_pv_mode()
        self._write_i32(OD_TARGET_VEL, 0)
        self.logger.info("Lift CAN command: stop target_velocity=0")

    def get_status(self):
        position = self._read_i32(OD_POS_ACTUAL)
        velocity = self._read_i32(OD_VEL_ACTUAL)
        status_word = self._read_u16(OD_STATUS_WORD)
        error_code = self._read_u16(OD_ERROR_CODE)
        mode_display = self._read_i8(OD_MODE_DISPLAY)
        raw_height = position / self.pulse_per_mm + self.height_offset
        raw_span = self.raw_height_max - self.raw_height_min
        if abs(raw_span) > 1e-6:
            normalized_height = (raw_height - self.raw_height_min) / raw_span
            normalized_height = min(max(normalized_height, 0.0), 1.0)
            height = normalized_height * self.max_height
        else:
            height = raw_height
            normalized_height = min(max(height / self.max_height, 0.0), 1.0)
        return {
            "height": height,
            "normalized_height": normalized_height,
            "raw_height": raw_height,
            "position": position,
            "velocity": velocity,
            "status_word": status_word,
            "error_code": error_code,
            "mode_display": mode_display,
        }

    def _prepare_pv_mode(self):
        self._send_nmt(0x01)
        self._fault_reset_if_needed()
        self._enable_operation()
        self._write_i8(OD_MODE, MODE_PV)
        mode_display = self._read_i8(OD_MODE_DISPLAY)
        if mode_display != MODE_PV:
            self.logger.warning(
                f"Lift mode display is {mode_display}, expected PV mode {MODE_PV}"
            )

    def _fault_reset_if_needed(self):
        status = self._read_u16(OD_STATUS_WORD)
        if (status >> 3) & 0x01:
            self._write_u16(OD_CONTROL_WORD, 0x0080)
            time.sleep(0.2)

    def _enable_operation(self):
        for control_word in (0x0006, 0x0007, 0x000F):
            self._write_u16(OD_CONTROL_WORD, control_word)
            time.sleep(0.05)

    def _send_nmt(self, command: int):
        self._send_frame(0x000, bytes([command, self.node_id]))

    def _read_u16(self, idx_sub):
        return self._sdo_upload(idx_sub[0], idx_sub[1], signed=False)

    def _read_i8(self, idx_sub):
        return self._sdo_upload(idx_sub[0], idx_sub[1], signed=True)

    def _read_i32(self, idx_sub):
        return self._sdo_upload(idx_sub[0], idx_sub[1], signed=True)

    def _write_u16(self, idx_sub, value: int):
        self._sdo_download(idx_sub[0], idx_sub[1], int(value), size=2, signed=False)

    def _write_i8(self, idx_sub, value: int):
        self._sdo_download(idx_sub[0], idx_sub[1], int(value), size=1, signed=True)

    def _write_i32(self, idx_sub, value: int):
        self._sdo_download(idx_sub[0], idx_sub[1], int(value), size=4, signed=True)

    def _sdo_upload(self, index: int, subindex: int, signed: bool):
        request = bytes(
            [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        )
        response = self._sdo_request(request)
        command = response[0]
        if command == 0x80:
            abort_code = int.from_bytes(response[4:8], "little", signed=False)
            raise RuntimeError(
                f"SDO upload abort index=0x{index:04X}, abort=0x{abort_code:08X}"
            )
        if command == 0x4F:
            size = 1
        elif command == 0x4B:
            size = 2
        elif command == 0x43:
            size = 4
        else:
            raise RuntimeError(
                f"Unexpected SDO upload response command=0x{command:02X}"
            )
        return int.from_bytes(response[4 : 4 + size], "little", signed=signed)

    def _sdo_download(
        self, index: int, subindex: int, value: int, size: int, signed: bool
    ):
        command_by_size = {1: 0x2F, 2: 0x2B, 4: 0x23}
        payload = int(value).to_bytes(size, "little", signed=signed)
        request = bytes(
            [command_by_size[size], index & 0xFF, (index >> 8) & 0xFF, subindex]
        )
        request += payload.ljust(4, b"\x00")
        response = self._sdo_request(request)
        if response[0] == 0x80:
            abort_code = int.from_bytes(response[4:8], "little", signed=False)
            raise RuntimeError(
                f"SDO download abort index=0x{index:04X}, abort=0x{abort_code:08X}"
            )
        if response[0] != 0x60:
            raise RuntimeError(
                f"Unexpected SDO download response command=0x{response[0]:02X}"
            )

    def _sdo_request(self, payload: bytes):
        request_id = 0x600 + self.node_id
        response_id = 0x580 + self.node_id
        self._send_frame(request_id, payload)

        deadline = time.time() + self.sdo_timeout_s
        while time.time() < deadline:
            can_id, data = self._recv_frame()
            if can_id == response_id:
                return data
        raise TimeoutError(f"Timed out waiting for SDO response from node {self.node_id}")

    def _send_frame(self, can_id: int, data: bytes):
        if self.socket is None:
            raise RuntimeError("CAN socket is not connected")
        data = bytes(data)
        frame = struct.pack("=IB3x8s", int(can_id), len(data), data.ljust(8, b"\x00"))
        self.socket.send(frame)

    def _recv_frame(self):
        if self.socket is None:
            raise RuntimeError("CAN socket is not connected")
        frame = self.socket.recv(16)
        can_id, can_dlc, data = struct.unpack("=IB3x8s", frame)
        return can_id & 0x1FFFFFFF, data[:can_dlc]


class LiftJoystickController(Node):
    STATE_STOP = 0
    STATE_RISE = 1
    STATE_DECLINE = -1

    def __init__(self):
        super().__init__("lift_joystick_controller")

        self.declare_parameter("enable_lift", True)
        self.declare_parameter("lift_joystick_deadzone", 0.35)
        self.declare_parameter("lift_command_timeout_s", 0.3)
        self.declare_parameter("lift_command_publish_rate", 10.0)
        self.declare_parameter("lift_status_rate", 1.0)
        self.declare_parameter("lift_joystick_poll_rate_hz", 30.0)
        self.declare_parameter("lift_invert_y", False)
        self.declare_parameter("lift_debug_log", False)
        self.declare_parameter("lift_status_log", False)

        self.declare_parameter("lift_can_interface", "can3")
        self.declare_parameter("lift_can_bitrate", 1000000)
        self.declare_parameter("lift_node_id", 16)
        self.declare_parameter("lift_velocity_command", 50000)
        self.declare_parameter("lift_max_velocity_command", 60000)
        self.declare_parameter("lift_sdo_timeout_s", 0.3)
        self.declare_parameter("lift_pulse_per_mm", 100.0)
        self.declare_parameter("lift_max_height", 600.0)
        self.declare_parameter("lift_height_offset", 600.0)
        self.declare_parameter("lift_raw_height_min", -6084.38)
        self.declare_parameter("lift_raw_height_max", 5132.72)

        self.declare_parameter("lift_command_topic", "/lift/command")
        self.declare_parameter("lift_height_topic", "/lift/height")
        self.declare_parameter("lift_normalized_height_topic", "/lift/normalized_height")
        self.declare_parameter("lift_position_topic", "/lift/position")
        self.declare_parameter("lift_velocity_topic", "/lift/velocity")
        self.declare_parameter("lift_status_word_topic", "/lift/status_word")
        self.declare_parameter("lift_error_code_topic", "/lift/error_code")
        self.declare_parameter("lift_mode_display_topic", "/lift/mode_display")

        self.declare_parameter("oculus_ip_address", "")
        self.declare_parameter("oculus_port", 5555)

        self.enabled = bool(self.get_parameter("enable_lift").value)
        self.deadzone = abs(float(self.get_parameter("lift_joystick_deadzone").value))
        self.command_timeout_s = float(
            self.get_parameter("lift_command_timeout_s").value
        )
        self.publish_rate_hz = float(
            self.get_parameter("lift_command_publish_rate").value
        )
        self.status_rate_hz = float(self.get_parameter("lift_status_rate").value)
        self.poll_rate_hz = float(self.get_parameter("lift_joystick_poll_rate_hz").value)
        self.invert_y = bool(self.get_parameter("lift_invert_y").value)
        self.debug_log = bool(self.get_parameter("lift_debug_log").value)
        self.status_log = bool(self.get_parameter("lift_status_log").value)

        self.can_interface = str(self.get_parameter("lift_can_interface").value)
        self.can_bitrate = int(self.get_parameter("lift_can_bitrate").value)
        self.node_id = int(self.get_parameter("lift_node_id").value)
        requested_velocity = int(self.get_parameter("lift_velocity_command").value)
        self.max_velocity_command = abs(
            int(self.get_parameter("lift_max_velocity_command").value)
        )
        self.velocity_command = min(abs(requested_velocity), self.max_velocity_command)
        self.sdo_timeout_s = float(self.get_parameter("lift_sdo_timeout_s").value)
        self.pulse_per_mm = float(self.get_parameter("lift_pulse_per_mm").value)
        self.max_height = float(self.get_parameter("lift_max_height").value)
        self.height_offset = float(self.get_parameter("lift_height_offset").value)
        self.raw_height_min = float(self.get_parameter("lift_raw_height_min").value)
        self.raw_height_max = float(self.get_parameter("lift_raw_height_max").value)

        self.command_topic = str(self.get_parameter("lift_command_topic").value)
        self.height_topic = str(self.get_parameter("lift_height_topic").value)
        self.normalized_height_topic = str(
            self.get_parameter("lift_normalized_height_topic").value
        )
        self.position_topic = str(self.get_parameter("lift_position_topic").value)
        self.velocity_topic = str(self.get_parameter("lift_velocity_topic").value)
        self.status_word_topic = str(self.get_parameter("lift_status_word_topic").value)
        self.error_code_topic = str(self.get_parameter("lift_error_code_topic").value)
        self.mode_display_topic = str(self.get_parameter("lift_mode_display_topic").value)
        self.oculus_ip_address = str(self.get_parameter("oculus_ip_address").value).strip()
        self.oculus_port = int(self.get_parameter("oculus_port").value)

        self.state = self.STATE_STOP
        self.axis_x = 0.0
        self.axis_y = 0.0
        self.last_update_time = time.monotonic()
        self._last_published_state = None
        self._throttle_times = {}
        self._io_lock = threading.Lock()
        self._command_condition = threading.Condition()
        self._pending_can_state = None
        self._stop_event = threading.Event()
        self._worker = None
        self._shutdown_done = False
        self.operator = None
        self.oculus_reader = None

        command_qos = QoSProfile(depth=1)
        command_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.command_pub = self.create_publisher(Int8, self.command_topic, command_qos)
        self.height_pub = self.create_publisher(Float32, self.height_topic, 1)
        self.normalized_height_pub = self.create_publisher(
            Float32, self.normalized_height_topic, 1
        )
        self.position_pub = self.create_publisher(Int32, self.position_topic, 1)
        self.velocity_pub = self.create_publisher(Int32, self.velocity_topic, 1)
        self.status_word_pub = self.create_publisher(UInt16, self.status_word_topic, 1)
        self.error_code_pub = self.create_publisher(UInt16, self.error_code_topic, 1)
        self.mode_display_pub = self.create_publisher(Int8, self.mode_display_topic, 1)

        self.watchdog = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0), self._watchdog_tick
        )
        self.status_timer = self.create_timer(
            1.0 / max(self.status_rate_hz, 0.1), self._publish_status_tick
        )
        self.button_timer = self.create_timer(
            1.0 / max(self.poll_rate_hz, 1.0), self._poll_oculus_buttons
        )

        if self.enabled:
            self._log_can_interface_status()
            self._init_lift_operator()
            self._init_oculus_reader()
            self._worker = threading.Thread(
                target=self._can_worker_loop,
                name="lift_canopen_controller",
                daemon=True,
            )
            self._worker.start()
            self.get_logger().info(
                "Lift joystick initialized: "
                f"command={self.command_topic}, height={self.height_topic}, "
                f"normalized={self.normalized_height_topic}, can={self.can_interface}, "
                f"node_id={self.node_id}, velocity_command={self.velocity_command}"
            )
        else:
            self.get_logger().info("Lift joystick control disabled by enable_lift=false")
            self._publish_state(self.STATE_STOP, force=True)

    def _init_oculus_reader(self):
        ip_address = self.oculus_ip_address or None
        try:
            from oculus_reader import OculusReader

            self.oculus_reader = OculusReader(
                ip_address=ip_address,
                port=self.oculus_port,
            )
        except BaseException as exc:
            self.oculus_reader = None
            self.get_logger().error(f"Failed to initialize OculusReader for lift: {exc}")

    def _init_lift_operator(self):
        try:
            self.operator = SimpleCanopenLift(
                logger=self.get_logger(),
                can_interface=self.can_interface,
                node_id=self.node_id,
                bitrate=self.can_bitrate,
                velocity_command=self.velocity_command,
                sdo_timeout_s=self.sdo_timeout_s,
                pulse_per_mm=self.pulse_per_mm,
                max_height=self.max_height,
                height_offset=self.height_offset,
                raw_height_min=self.raw_height_min,
                raw_height_max=self.raw_height_max,
            )
            self.operator.connect()
        except Exception as exc:
            self.operator = None
            self.get_logger().error(
                f"Failed to initialize lift CANopen on {self.can_interface}: {exc}"
            )

    def _log_can_interface_status(self):
        result = subprocess.run(
            ["ip", "link", "show", self.can_interface],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            self.get_logger().info(f"Lift CAN interface {self.can_interface} exists.")
        else:
            self.get_logger().warning(
                f"Lift CAN interface {self.can_interface} was not found. "
                "Check CAN activation before moving the lift."
            )

    def _poll_oculus_buttons(self):
        if not self.enabled or self.oculus_reader is None:
            return
        _, buttons = self.oculus_reader.get_transformations_and_buttons()
        self.update(buttons)

    def _desired_state_from_buttons(self, buttons):
        self.axis_x = 0.0
        self.axis_y = 0.0

        if not isinstance(buttons, dict):
            return self.STATE_STOP

        left_js = buttons.get("leftJS")
        if not isinstance(left_js, (tuple, list)) or len(left_js) < 2:
            return self.STATE_STOP

        self.axis_x = float(left_js[0])
        self.axis_y = float(left_js[1])
        axis_y = -self.axis_y if self.invert_y else self.axis_y

        if axis_y > self.deadzone:
            return self.STATE_RISE
        if axis_y < -self.deadzone:
            return self.STATE_DECLINE
        return self.STATE_STOP

    def _publish_state(self, state: int, force: bool = False):
        if not force and state == self._last_published_state:
            return

        self.command_pub.publish(Int8(data=int(state)))
        self._last_published_state = state
        self.get_logger().info(
            f"Lift command published: {self._state_name(state)}({state}), "
            f"topic={self.command_topic}, lift_can={self.can_interface}"
        )
        self._queue_lift_can_command(state)

    def _queue_lift_can_command(self, state: int):
        if not self.enabled:
            return
        with self._command_condition:
            self._pending_can_state = state
            self._command_condition.notify()

    def _can_worker_loop(self):
        while not self._stop_event.is_set():
            with self._command_condition:
                while self._pending_can_state is None and not self._stop_event.is_set():
                    self._command_condition.wait(timeout=0.1)
                if self._stop_event.is_set():
                    break
                state = self._pending_can_state
                self._pending_can_state = None
            self._send_lift_can_command(state)

    def _send_lift_can_command(self, state: int):
        if self.operator is None:
            self._log_throttled(
                "operator_missing",
                1.0,
                "error",
                f"Lift CAN command not sent because CANopen operator is not initialized. "
                f"interface={self.can_interface}",
            )
            return

        try:
            with self._io_lock:
                if state == self.STATE_RISE:
                    self.operator.control_rise()
                elif state == self.STATE_DECLINE:
                    self.operator.control_decline()
                else:
                    self.operator.control_stop()
        except Exception as exc:
            self._log_throttled(
                "send_failed",
                1.0,
                "error",
                f"Failed to send lift CAN command {self._state_name(state)}({state}) "
                f"on {self.can_interface}: {exc}",
            )

    def _publish_status_tick(self):
        if not self.enabled or self.operator is None:
            return
        try:
            with self._io_lock:
                status = self.operator.get_status()
        except Exception as exc:
            self._log_throttled(
                "status_failed", 1.0, "warning", f"Failed to read lift status: {exc}"
            )
            return

        self.height_pub.publish(Float32(data=float(status["height"])))
        self.normalized_height_pub.publish(
            Float32(data=float(status["normalized_height"]))
        )
        self.position_pub.publish(Int32(data=int(status["position"])))
        self.velocity_pub.publish(Int32(data=int(status["velocity"])))
        self.status_word_pub.publish(UInt16(data=int(status["status_word"])))
        self.error_code_pub.publish(UInt16(data=int(status["error_code"])))
        self.mode_display_pub.publish(Int8(data=int(status["mode_display"])))
        if self.status_log:
            self._log_throttled(
                "status",
                1.0,
                "info",
                "Lift status: "
                f"height={status['height']:.2f}mm "
                f"normalized={status['normalized_height']:.3f} "
                f"pos={status['position']} vel={status['velocity']} "
                f"status=0x{status['status_word']:04X} "
                f"error=0x{status['error_code']:04X} "
                f"mode={status['mode_display']}",
            )

    def _state_name(self, state: int) -> str:
        return {
            self.STATE_RISE: "rise",
            self.STATE_DECLINE: "decline",
            self.STATE_STOP: "stop",
        }.get(state, "unknown")

    def _log_debug(self, buttons):
        if not self.debug_log:
            return

        left_js = buttons.get("leftJS") if isinstance(buttons, dict) else None
        if not isinstance(left_js, (tuple, list)) or len(left_js) < 2:
            self._log_throttled(
                "debug_missing",
                1.0,
                "warning",
                f"Lift debug: no valid leftJS data, command={self._state_name(self.state)}"
                f"({self.state}), topic={self.command_topic}",
            )
            return

        self._log_throttled(
            "debug",
            0.5,
            "info",
            f"Lift debug: leftJS=({self.axis_x:.3f}, {self.axis_y:.3f}), "
            f"deadzone={self.deadzone:.3f}, command={self._state_name(self.state)}"
            f"({self.state}), topic={self.command_topic}, lift_can={self.can_interface}",
        )

    def update(self, buttons):
        self.last_update_time = time.monotonic()

        desired_state = self._desired_state_from_buttons(buttons)
        if not self.enabled:
            desired_state = self.STATE_STOP

        if desired_state != self.state:
            self.state = desired_state
            self._publish_state(self.state, force=True)

        self._log_debug(buttons)

    def _watchdog_tick(self):
        if time.monotonic() - self.last_update_time > self.command_timeout_s:
            self.state = self.STATE_STOP
        self._publish_state(self.state)

    def _log_throttled(self, key: str, period_s: float, level: str, message: str):
        now = time.monotonic()
        last_time = self._throttle_times.get(key)
        if last_time is not None and now - last_time < period_s:
            return
        self._throttle_times[key] = now
        getattr(self.get_logger(), level)(message)

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self.state = self.STATE_STOP
        if rclpy.ok():
            try:
                self._publish_state(self.STATE_STOP, force=True)
            except Exception as exc:
                self.get_logger().warning(f"Failed to publish lift stop during shutdown: {exc}")
        self._stop_event.set()
        with self._command_condition:
            self._command_condition.notify()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
        if self.operator is not None:
            try:
                self.operator.close()
            except Exception as exc:
                self.get_logger().warning(f"Failed to close lift CANopen cleanly: {exc}")
        if self.oculus_reader is not None:
            self.oculus_reader.stop()

        for timer in (self.button_timer, self.status_timer, self.watchdog):
            timer.cancel()

    def destroy_node(self):
        self.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiftJoystickController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
