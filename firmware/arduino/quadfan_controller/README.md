# Quadfan Arduino Controller

## Wiring (example)
- **MPU6050**: SDA -> A4, SCL -> A5 (UNO), VCC -> 5V, GND -> GND.
- **Motors (ESCs or duct fans with PWM)**:
  - Front-left: pin 3
  - Front-right: pin 5
  - Rear-left: pin 6
  - Rear-right: pin 9

Pins are configurable at the top of `quadfan_controller.ino`.

## Safety
- This firmware **starts DISARMED**. Motors stay off until `ARM 1`.
- If no valid `CMD` arrives within the watchdog timeout (default 500 ms), motors stop.
- `STOP` immediately kills motors even while armed.
- For bench testing, **remove props** and keep the rig tethered.

## Serial protocol (USB)
- `ARM 1` / `ARM 0`
- `STOP`
- `CMD <mode> <throttle_0_255> <pitch_deg> <roll_deg>`
  - mode: `TETHERED` or `UNTETHERED`

Telemetry at ~10 Hz:
- `ATT <pitch_deg> <roll_deg> <armed> <mode>`

## Notes
- UNTETHERED mode uses an MPU6050 IMU + complementary filter + PID.
- TETHERED mode uses open-loop mixing.
- Mixer (X):
  - FL = T + P + R
  - FR = T + P - R
  - RL = T - P + R
  - RR = T - P - R
