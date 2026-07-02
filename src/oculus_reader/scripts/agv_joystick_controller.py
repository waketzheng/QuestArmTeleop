#!/usr/bin/env python3
import json
import threading
import time
from urllib.error import HTTPError
from urllib.parse import urljoin
from urllib.request import Request, urlopen

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class HttpAGVOperator:
    def __init__(
        self,
        logger,
        ip: str,
        port: int,
        request_timeout_s: float = 5.0,
        stop_path: str = "/cmd/stop",
    ):
        self.logger = logger
        self.ip = ip.strip().rstrip("/")
        self.port = int(port)
        self.request_timeout_s = max(0.1, float(request_timeout_s))
        stop_path = stop_path.strip() or "/cmd/stop"
        self.stop_path = stop_path if stop_path.startswith("/") else f"/{stop_path}"

    @property
    def host(self):
        if self.ip.startswith(("http://", "https://")):
            if self.port in (80, 443):
                return self.ip
            return f"{self.ip}:{self.port}"
        return f"http://{self.ip}:{self.port}"

    def get_speed(self):
        return self._request("/reeman/speed", "GET")

    def move(self, direction: int, distance: int, speed: float):
        return self._request(
            "/cmd/move",
            "POST",
            data={
                "direction": int(direction),
                "distance": max(1, int(distance)),
                "speed": max(0.01, float(speed)),
            },
        )

    def turn(self, direction: int, angle: int, speed: float):
        return self._request(
            "/cmd/turn",
            "POST",
            data={
                "direction": int(direction),
                "angle": max(1, int(angle)),
                "speed": max(0.01, float(speed)),
            },
        )

    def stop(self):
        return self._request(self.stop_path, "POST")

    def _request(self, path: str, method: str, data=None):
        url = urljoin(self.host, path)
        body = None
        if data is not None:
            body = json.dumps(data).encode("utf-8")

        request = Request(
            url,
            data=body,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urlopen(request, timeout=self.request_timeout_s) as response:
                text = response.read().decode("utf-8")
                status = response.status
            if status != 200:
                self.logger.warning(
                    f"AGV HTTP {method} {url} failed: "
                    f"status={status}, body={text}"
                )
                return None
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                return {"text": text}
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            self.logger.warning(
                f"AGV HTTP {method} {url} failed: status={exc.code}, body={body}"
            )
            return None
        except Exception as exc:
            self.logger.warning(f"AGV HTTP {method} {url} error: {exc}")
            return None


class AGVJoystickController(Node):
    def __init__(self):
        super().__init__("agv_joystick_controller")

        self.declare_parameter("enable_agv", True)
        self.declare_parameter("agv_joystick_deadzone", 0.35)
        self.declare_parameter("agv_command_timeout_s", 0.4)
        self.declare_parameter("agv_idle_poll_interval_s", 0.05)
        self.declare_parameter("agv_idle_timeout_s", 1.5)
        self.declare_parameter("agv_forward_distance_cm", 20)
        self.declare_parameter("agv_backward_distance_cm", 20)
        self.declare_parameter("agv_arc_distance_cm", 10)
        self.declare_parameter("agv_arc_turn_angle_deg", 15)
        self.declare_parameter("agv_command_pulse_s", 0.2)
        self.declare_parameter("agv_stop_path", "/cmd/stop")
        self.declare_parameter("agv_move_speed", 0.15)
        self.declare_parameter("agv_turn_speed", 0.35)
        self.declare_parameter("agv_sample_rate", 10)
        self.declare_parameter("agv_invert_x", False)
        self.declare_parameter("agv_invert_y", False)
        self.declare_parameter("agv_debug_log", False)
        self.declare_parameter("agv_request_timeout_s", 5.0)
        self.declare_parameter("agv_ip", "http://192.168.1.100/")
        self.declare_parameter("agv_port", 80)
        self.declare_parameter("oculus_ip_address", "")
        self.declare_parameter("oculus_port", 5555)

        self.enabled = bool(self.get_parameter("enable_agv").value)
        self.deadzone = abs(float(self.get_parameter("agv_joystick_deadzone").value))
        self.command_timeout_s = float(
            self.get_parameter("agv_command_timeout_s").value
        )
        self.idle_poll_interval_s = float(
            self.get_parameter("agv_idle_poll_interval_s").value
        )
        self.idle_timeout_s = float(self.get_parameter("agv_idle_timeout_s").value)
        self.forward_distance_cm = int(
            self.get_parameter("agv_forward_distance_cm").value
        )
        self.backward_distance_cm = int(
            self.get_parameter("agv_backward_distance_cm").value
        )
        self.arc_distance_cm = int(self.get_parameter("agv_arc_distance_cm").value)
        self.arc_turn_angle_deg = int(
            self.get_parameter("agv_arc_turn_angle_deg").value
        )
        self.stop_path = (
            str(self.get_parameter("agv_stop_path").value).strip() or "/cmd/stop"
        )
        self.move_speed = float(self.get_parameter("agv_move_speed").value)
        self.turn_speed = float(self.get_parameter("agv_turn_speed").value)
        self.sample_rate = int(self.get_parameter("agv_sample_rate").value)
        self.invert_x = bool(self.get_parameter("agv_invert_x").value)
        self.invert_y = bool(self.get_parameter("agv_invert_y").value)
        self.debug_log = bool(self.get_parameter("agv_debug_log").value)
        self.request_timeout_s = float(self.get_parameter("agv_request_timeout_s").value)
        self.ip = str(self.get_parameter("agv_ip").value)
        self.port = int(self.get_parameter("agv_port").value)
        self.oculus_ip_address = str(self.get_parameter("oculus_ip_address").value).strip()
        self.oculus_port = int(self.get_parameter("oculus_port").value)

        self.operator = None
        self.oculus_reader = None
        self.axis_x = 0.0
        self.axis_y = 0.0
        self.last_update_time = time.monotonic()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker = None
        self._shutdown_done = False
        self._throttle_times = {}
        self._was_active = False

        if not self.enabled:
            self.get_logger().info("AGV joystick control disabled by enable_agv=false")
            self.button_timer = self.create_timer(1.0, self._poll_oculus_buttons)
            return

        try:
            self.operator = HttpAGVOperator(
                logger=self.get_logger(),
                ip=self.ip,
                port=self.port,
                request_timeout_s=self.request_timeout_s,
                stop_path=self.stop_path,
            )
            self._init_oculus_reader()
            self._worker = threading.Thread(
                target=self._worker_loop,
                name="agv_joystick_controller",
                daemon=True,
            )
            self._worker.start()
            self.button_timer = self.create_timer(
                1.0 / max(float(self.sample_rate), 1.0), self._poll_oculus_buttons
            )
            self.get_logger().info(
                "AGV joystick control initialized: "
                f"ip={self.ip}, port={self.port}, deadzone={self.deadzone:.2f}, "
                f"move_distance=(forward={self.forward_distance_cm}cm, "
                f"backward={self.backward_distance_cm}cm), "
                f"turn_angle={self.arc_turn_angle_deg}deg, "
                f"stop_path={self.stop_path}"
            )
        except Exception as exc:
            self.operator = None
            self.button_timer = self.create_timer(1.0, self._poll_oculus_buttons)
            self.get_logger().error(f"Failed to initialize AGV joystick control: {exc}")

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
            self.get_logger().error(f"Failed to initialize OculusReader for AGV: {exc}")

    def _poll_oculus_buttons(self):
        if not self.enabled or self.oculus_reader is None:
            return
        _, buttons = self.oculus_reader.get_transformations_and_buttons()
        self.update(buttons)

    def _extract_axes(self, buttons):
        if not isinstance(buttons, dict):
            return 0.0, 0.0

        right_js = buttons.get("rightJS")
        if not isinstance(right_js, (tuple, list)) or len(right_js) < 2:
            return 0.0, 0.0

        axis_x = float(right_js[0])
        axis_y = float(right_js[1])
        if self.invert_x:
            axis_x = -axis_x
        if self.invert_y:
            axis_y = -axis_y

        if abs(axis_x) < self.deadzone:
            axis_x = 0.0
        if abs(axis_y) < self.deadzone:
            axis_y = 0.0
        return axis_x, axis_y

    def update(self, buttons):
        if self.operator is None:
            return

        axis_x, axis_y = self._extract_axes(buttons)
        if self.debug_log:
            right_js = buttons.get("rightJS") if isinstance(buttons, dict) else None
            self._log_throttled(
                "debug",
                0.5,
                "info",
                f"AGV joystick debug: raw_rightJS={right_js}, "
                f"filtered_axis=(x={axis_x:.3f}, y={axis_y:.3f}), "
                f"deadzone={self.deadzone:.3f}",
            )
        with self._lock:
            self.last_update_time = time.monotonic()
            self.axis_x = axis_x
            self.axis_y = axis_y

    def _wait_until_idle(self, timeout_s: float):
        if self.operator is None:
            return False

        stable = 0
        start_time = time.monotonic()
        while (
            time.monotonic() - start_time < timeout_s
            and not self._stop_event.is_set()
        ):
            idle_context = self._current_idle_context()
            if idle_context is not None and self._was_active:
                self._send_stop(idle_context)
                self._was_active = False
                return False

            speed = self.operator.get_speed()
            if speed is not None:
                vx = abs(float(speed.get("vx", 0.0)))
                vth = abs(float(speed.get("vth", 0.0)))
                if vx <= 0.02 and vth <= 0.03:
                    stable += 1
                    if stable >= 2:
                        return True
                else:
                    stable = 0
            time.sleep(self.idle_poll_interval_s)
        return False

    def _log_speed_snapshot(self, context: str):
        if self.operator is None:
            return
        speed = self.operator.get_speed()
        if speed is None:
            self.get_logger().warning(f"AGV speed feedback after {context}: unavailable")
            return
        self.get_logger().info(f"AGV speed feedback after {context}: {speed}")

    def _send_move(self, direction: int, distance_cm: int):
        if self.operator is None:
            return

        self.get_logger().info(
            f"AGV move command: direction={direction}, "
            f"distance={max(1, int(distance_cm))}cm, "
            f"speed={max(0.01, self.move_speed):.3f}m/s"
        )
        resp = self.operator.move(
            direction=direction,
            distance=max(1, int(distance_cm)),
            speed=max(0.01, self.move_speed),
        )
        if resp is None:
            self.get_logger().warning("AGV move command returned None")
        elif self.debug_log:
            self.get_logger().info(f"AGV move response: {resp}")
        self._log_speed_snapshot("move")

        est_duration = max(
            0.15, float(distance_cm) / 100.0 / max(self.move_speed, 0.01)
        )
        self._wait_until_idle(max(self.idle_timeout_s, est_duration * 2.0))

    def _send_turn(self, direction: int, angle_deg: int):
        if self.operator is None:
            return

        self.get_logger().info(
            f"AGV turn command: direction={direction}, "
            f"angle={max(1, int(angle_deg))}deg, "
            f"speed={max(0.01, self.turn_speed):.3f}rad/s"
        )
        resp = self.operator.turn(
            direction=direction,
            angle=max(1, int(angle_deg)),
            speed=max(0.01, self.turn_speed),
        )
        if resp is None:
            self.get_logger().warning("AGV turn command returned None")
        elif self.debug_log:
            self.get_logger().info(f"AGV turn response: {resp}")
        self._log_speed_snapshot("turn")

        est_duration = max(
            0.15,
            angle_deg * 3.141592653589793 / 180.0 / max(self.turn_speed, 0.01),
        )
        self._wait_until_idle(max(self.idle_timeout_s, est_duration * 2.0))

    def _send_stop(self, context: str):
        if self.operator is None:
            return

        resp = self.operator.stop()
        if resp is None:
            self._log_throttled(
                "agv_stop_failed",
                1.0,
                "warning",
                f"AGV stop command returned None during {context}. "
                f"Check agv_stop_path={self.stop_path}",
            )
        elif self.debug_log:
            self.get_logger().info(f"AGV stop response during {context}: {resp}")

    def _current_idle_context(self):
        with self._lock:
            axis_x = self.axis_x
            axis_y = self.axis_y
            age_s = time.monotonic() - self.last_update_time

        if age_s > self.command_timeout_s:
            return "input timeout"
        if axis_x == 0.0 and axis_y == 0.0:
            return "joystick idle"
        return None

    def _execute_control_cycle(self, axis_x: float, axis_y: float):
        turning_active = axis_x != 0.0
        moving_active = axis_y != 0.0

        if turning_active:
            turn_direction = 1 if axis_x < 0.0 else 0
            if self.debug_log:
                self.get_logger().info(
                    "AGV joystick action: "
                    f"axis_x={axis_x:.3f} -> "
                    f"{'left' if turn_direction == 1 else 'right'} turn"
                )
            self._send_turn(
                direction=turn_direction,
                angle_deg=max(1, int(self.arc_turn_angle_deg)),
            )
            if self._stop_event.is_set():
                return

        if moving_active:
            move_direction = 1 if axis_y > 0.0 else 0
            distance_cm = (
                self.forward_distance_cm
                if move_direction == 1
                else self.backward_distance_cm
            )
            if turning_active:
                distance_cm = self.arc_distance_cm
            distance_cm = max(1, int(distance_cm))
            if self.debug_log:
                self.get_logger().info(
                    "AGV joystick action: "
                    f"axis_y={axis_y:.3f} -> "
                    f"{'forward' if move_direction == 1 else 'backward'} move, "
                    f"distance={distance_cm}cm"
                )
            self._send_move(direction=move_direction, distance_cm=distance_cm)

    def _worker_loop(self):
        while not self._stop_event.is_set():
            with self._lock:
                axis_x = self.axis_x
                axis_y = self.axis_y
                age_s = time.monotonic() - self.last_update_time

            if age_s > self.command_timeout_s:
                axis_x = 0.0
                axis_y = 0.0

            if axis_x == 0.0 and axis_y == 0.0:
                if self._was_active:
                    self._send_stop(
                        "input timeout"
                        if age_s > self.command_timeout_s
                        else "joystick idle"
                    )
                    self._was_active = False
                time.sleep(0.03)
                continue

            self._was_active = True
            self._execute_control_cycle(axis_x, axis_y)

    def _log_throttled(self, key: str, period_s: float, level: str, message: str):
        now = time.monotonic()
        last_time = self._throttle_times.get(key)
        if last_time is not None and now - last_time < period_s:
            return
        self._throttle_times[key] = now
        logger = self.get_logger()
        if level == "debug":
            logger.debug(message)
        elif level == "info":
            logger.info(message)
        elif level == "warning":
            logger.warning(message)
        elif level == "error":
            logger.error(message)
        else:
            logger.info(message)

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
        if self.operator is not None:
            try:
                self.operator.stop()
            except Exception as exc:
                self.get_logger().warning(f"Failed to close AGV operator cleanly: {exc}")
        if self.oculus_reader is not None:
            self.oculus_reader.stop()
        self.button_timer.cancel()

    def destroy_node(self):
        self.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AGVJoystickController()
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
