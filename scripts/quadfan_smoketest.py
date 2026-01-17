import time
import argparse
import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Quadfan Arduino serial smoke test")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", default="TETHERED")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        def send(line: str) -> None:
            print(f">>> {line.strip()}")
            ser.write(line.encode("utf-8"))

        send("ARM 1\n")
        time.sleep(0.2)
        send(f"CMD {args.mode} 50 2 0\n")
        time.sleep(0.5)
        send("STOP\n")
        time.sleep(0.2)
        send("ARM 0\n")

        start = time.time()
        while time.time() - start < 2.0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"<<< {line}")


if __name__ == "__main__":
    main()
