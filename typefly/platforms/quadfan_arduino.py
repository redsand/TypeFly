import time
import threading
from typing import Any, Optional

import numpy as np
from PIL import Image
from overrides import overrides
import serial

from ..robot_wrapper import RobotWrapper, RobotObservation
from ..robot_info import RobotInfo
from ..utils import print_t

TELEMETRY_RATE_HZ = 10

class QuadfanObservation(RobotObservation):
    def __init__(self, robot_info: RobotInfo, rate: int = 2):
        super().__init__(robot_info, rate)
        self._image = Image.new("RGB", (640, 480), color=(10, 10, 10))

    @overrides
    def _start(self):
        return None

    @overrides
    def _stop(self):
        return None

    @overrides
    async def process_image(self, image: Image.Image):
        return None

    @overrides
    def fetch_processed_result(self) -> dict[str, Any]:
        return {}


class QuadfanArduinoWrapper(RobotWrapper):
    def __init__(self, robot_info: RobotInfo):
        extra = robot_info.extra or {}
        self.serial_port = extra.get("serial_port", "/dev/ttyACM0")
        self.baud = int(extra.get("baud", 115200))
        self.mode = str(extra.get("mode_default", "TETHERED")).upper()
        self.throttle_max_tethered = float(extra.get("throttle_max_tethered", 0.35))
        self.throttle_max_untethered = float(extra.get("throttle_max_untethered", 0.60))
        self.watchdog_ms = int(extra.get("watchdog_ms", 500))
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._telemetry_thread = None
        self._running = False
        self._armed = False
        self._last_cmd = None
        self._last_attitude = (0.0, 0.0)

        super().__init__(robot_info, QuadfanObservation(robot_info))

        self.skillset.add_skill(self.arm, "Arm the quadfan (motors enabled)")
        self.skillset.add_skill(self.disarm, "Disarm the quadfan (motors off)")
        self.skillset.add_skill(self.stop, "Hard stop the motors immediately")
        self.skillset.add_skill(self.set_throttle, "Set throttle 0..1 (maintains current mode)")
        self.skillset.add_skill(self.set_attitude, "Set pitch/roll targets and throttle")
        self.skillset.add_skill(self.hold, "Hold the last command for seconds")
        self.skillset.add_skill(self.set_mode, "Set operating mode: TETHERED or UNTETHERED")
        self.skillset.add_skill(self.get_attitude, "Get latest pitch/roll telemetry")

    @overrides
    def start(self) -> bool:
        if not self._connect():
            return False
        self.obs.start()
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        self.disarm()
        self.stop()
        return True

    @overrides
    def stop(self) -> bool:
        self._send_line("STOP\n")
        self._last_cmd = None
        return True

    def shutdown(self) -> bool:
        self._running = False
        self.stop()
        self.disarm()
        if self._telemetry_thread:
            self._telemetry_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.obs.stop()
        return True

    @overrides
    def _move(self, dx: float, dy: float):
        print_t(f"[quadfan] move not supported (dx={dx}, dy={dy})")

    @overrides
    def _rotate(self, deg: float):
        print_t(f"[quadfan] rotate not supported (deg={deg})")

    def _connect(self) -> bool:
        try:
            self._serial = serial.Serial(self.serial_port, self.baud, timeout=0.1)
            print_t(f"[quadfan] Connected to {self.serial_port} @ {self.baud}")
            return True
        except serial.SerialException as exc:
            print_t(f"[quadfan] Serial connection failed: {exc}")
            return False

    def _ensure_connection(self) -> bool:
        if self._serial and self._serial.is_open:
            return True
        return self._connect()

    def _send_line(self, line: str) -> bool:
        if not self._ensure_connection():
            return False
        with self._serial_lock:
            try:
                self._serial.write(line.encode("utf-8"))
                return True
            except serial.SerialException as exc:
                print_t(f"[quadfan] Serial write failed: {exc}")
                return False

    def _telemetry_loop(self):
        buffer = ""
        while self._running:
            if not self._ensure_connection():
                time.sleep(0.5)
                continue
            try:
                with self._serial_lock:
                    if self._serial.in_waiting:
                        chunk = self._serial.read(self._serial.in_waiting).decode("utf-8", errors="ignore")
                    else:
                        chunk = ""
                if chunk:
                    buffer += chunk
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        self._handle_line(line.strip())
                else:
                    time.sleep(1.0 / TELEMETRY_RATE_HZ)
            except serial.SerialException as exc:
                print_t(f"[quadfan] Serial read failed: {exc}")
                time.sleep(0.5)

    def _handle_line(self, line: str):
        if not line:
            return
        if line.startswith("ATT"):
            parts = line.split()
            if len(parts) >= 5:
                try:
                    pitch = float(parts[1])
                    roll = float(parts[2])
                    armed = parts[3] == "1"
                    mode = parts[4]
                except ValueError:
                    return
                self._last_attitude = (pitch, roll)
                self._armed = armed
                self.mode = mode
                self.obs._orientation = np.array([np.deg2rad(pitch), np.deg2rad(roll), 0.0])

    def arm(self):
        if self._send_line("ARM 1\n"):
            self._armed = True
            print_t("[quadfan] Armed")

    def disarm(self):
        if self._send_line("ARM 0\n"):
            self._armed = False
            print_t("[quadfan] Disarmed")

    def set_mode(self, mode: str):
        mode = mode.upper()
        if mode not in {"TETHERED", "UNTETHERED"}:
            raise ValueError("mode must be TETHERED or UNTETHERED")
        self.mode = mode
        print_t(f"[quadfan] Mode set to {self.mode}")

    def set_throttle(self, value: float):
        self.set_attitude(0.0, 0.0, value)

    def _clamp_throttle(self, value: float) -> float:
        value = max(0.0, min(1.0, float(value)))
        if self.mode == "TETHERED":
            return min(value, self.throttle_max_tethered)
        return min(value, self.throttle_max_untethered)

    def set_attitude(self, pitch_deg: float, roll_deg: float, throttle: float = 0.0):
        if not self._armed:
            print_t("[quadfan] Ignoring set_attitude while disarmed")
            self.stop()
            return
        throttle = self._clamp_throttle(throttle)
        pwm = int(throttle * 255)
        cmd = f"CMD {self.mode} {pwm} {pitch_deg:.2f} {roll_deg:.2f}\n"
        if self._send_line(cmd):
            self._last_cmd = (pitch_deg, roll_deg, throttle, time.time())

    def hold(self, seconds: float):
        if not self._last_cmd:
            time.sleep(seconds)
            return
        pitch_deg, roll_deg, throttle, _ = self._last_cmd
        end_time = time.time() + seconds
        interval = min(0.1, self.watchdog_ms / 1000.0 / 2)
        while time.time() < end_time:
            self.set_attitude(pitch_deg, roll_deg, throttle)
            time.sleep(interval)

    def get_attitude(self) -> tuple[float, float]:
        return self._last_attitude
