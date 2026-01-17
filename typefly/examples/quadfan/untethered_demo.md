# Quadfan Untethered Demo (Bench Only)

## Warnings
- UNTETHERED mode uses closed-loop PID stabilization on the Arduino.
- **Remove props** and keep the frame tethered for all testing.
- Make sure the IMU (MPU6050) is wired and responding before arming.

## Run
1. Set `mode_default` to `"UNTETHERED"` in `robot_info.json`.
2. Start the web UI:
   ```bash
   python -m typefly.webui
   ```
3. In the web UI chat, paste this script:

```
arm()
set_attitude(0, 0, 0.45)
hold(3)
set_attitude(8, 0, 0.45)
hold(1.5)
set_attitude(0, 0, 0.45)
hold(1.5)
stop()
disarm()
```

## Notes
- Use small setpoints (<= 10 degrees) on the bench.
- If the IMU is noisy, lower throttle or reduce PID gains.
