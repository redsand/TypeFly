# Quadfan Arduino Platform (quadfan_arduino)

## Overview
The quadfan_arduino platform controls a four duct-fan “quad” test rig over USB serial. It supports two modes:

- **TETHERED (science-fair safe)**: Open-loop mixer. The Python wrapper sends throttle + pitch_deg + roll_deg, and the Arduino maps them directly to motor PWM.
- **UNTETHERED (demo only)**: Closed-loop stabilization on the Arduino using an MPU6050 IMU and PID. The Python wrapper sends target pitch/roll plus throttle.

## Safety checklist
1. **Props removed for testing.**
2. **DISARMED by default.** You must explicitly `arm()`.
3. **Watchdog stop**: if no valid `CMD` for 500 ms, motors stop.
4. **Throttle clamp**: TETHERED defaults to 35% max (configurable).
5. **Kill switch**: `stop()` always works (sends `STOP`).

## Configuration (robot_info.json)
Set `robot_type` to `quadfan_arduino` and add the extra configuration block. Example:

```json
{
  "robot_id": "quadfan1",
  "robot_type": "quadfan_arduino",
  "extra": {
    "serial_port": "/dev/ttyACM0",
    "baud": 115200,
    "mode_default": "TETHERED",
    "throttle_max_tethered": 0.35,
    "throttle_max_untethered": 0.60,
    "pitch_gain": 4.0,
    "roll_gain": 4.0,
    "pid_pitch": {"kp": 2.0, "ki": 0.1, "kd": 0.05},
    "pid_roll": {"kp": 2.0, "ki": 0.1, "kd": 0.05},
    "watchdog_ms": 500,
    "motor_pins": {
      "front_left": 3,
      "front_right": 5,
      "rear_left": 6,
      "rear_right": 9
    }
  }
}
```

## Running TypeFly
1. Start the vision service (if you use YOLO):
   ```bash
   cd typefly/proto && bash generate.sh
   python -m typefly.serving
   ```
2. Start the web UI:
   ```bash
   python -m typefly.webui
   ```
3. In the UI chat, use the example scripts from `typefly/examples/quadfan/`.

## Notes
- For safety and stability, build a rigid frame (3D-printed or CNC) and use ducted fans with guards.
- UNTETHERED mode is a bench demo only; always remove props.
- See `firmware/arduino/quadfan_controller/README.md` for wiring and firmware setup.
