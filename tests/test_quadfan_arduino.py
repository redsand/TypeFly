import importlib.util
import math
import unittest
from typing import TYPE_CHECKING

from typefly.robot_info import RobotInfo

if TYPE_CHECKING:
    from typefly.platforms.quadfan_arduino import QuadfanArduinoWrapper

NUMPY_AVAILABLE = importlib.util.find_spec("numpy") is not None


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes = []
        self.in_waiting = 0

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def read(self, size: int) -> bytes:
        return b""

    def close(self) -> None:
        self.is_open = False


@unittest.skipUnless(NUMPY_AVAILABLE, "numpy is required for quadfan wrapper tests")
class QuadfanArduinoWrapperTests(unittest.TestCase):
    def make_wrapper(self, extra=None):
        from typefly.platforms.quadfan_arduino import QuadfanArduinoWrapper

        info = RobotInfo("quadfan1", "quadfan_arduino", extra or {})
        wrapper = QuadfanArduinoWrapper(info)
        wrapper._serial = FakeSerial()
        return wrapper

    def test_disarmed_set_attitude_sends_stop(self):
        wrapper = self.make_wrapper()
        wrapper._armed = False
        wrapper.set_attitude(5.0, 0.0, 0.5)
        self.assertEqual(wrapper._serial.writes[-1], b"STOP\n")
        self.assertFalse(any(b"CMD" in write for write in wrapper._serial.writes))

    def test_throttle_clamp_tethered(self):
        wrapper = self.make_wrapper({"throttle_max_tethered": 0.35})
        wrapper._armed = True
        wrapper.mode = "TETHERED"
        wrapper.set_attitude(0.0, 0.0, 1.0)
        self.assertTrue(wrapper._serial.writes)
        last_write = wrapper._serial.writes[-1].decode("utf-8")
        self.assertIn("CMD TETHERED 89", last_write)

    def test_handle_line_updates_attitude(self):
        wrapper = self.make_wrapper()
        wrapper._handle_line("ATT 1.50 -2.00 1 TETHERED")
        self.assertEqual(wrapper.get_attitude(), (1.50, -2.00))
        self.assertTrue(wrapper._armed)
        self.assertEqual(wrapper.mode, "TETHERED")
        self.assertTrue(math.isclose(wrapper.obs.orientation[0], math.radians(1.5), rel_tol=1e-6))
        self.assertTrue(math.isclose(wrapper.obs.orientation[1], math.radians(-2.0), rel_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
