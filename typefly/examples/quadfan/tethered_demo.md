# Quadfan Tethered Demo (Safe)

## Requirements
- Firmware flashed to the Arduino: `firmware/arduino/quadfan_controller/quadfan_controller.ino`
- `robot_info.json` set to `quadfan_arduino` with `mode_default: "TETHERED"`
- Props removed or ducted fans with guards

## Run
1. Start the web UI:
   ```bash
   python -m typefly.webui
   ```
2. In the web UI chat, paste this script:

```
arm()
set_attitude(5, 0, 0.25)
hold(2)
set_attitude(0, 5, 0.25)
hold(2)
stop()
disarm()
```

## Notes
- The tethered mode clamps throttle by `throttle_max_tethered` in config.
