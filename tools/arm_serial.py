"""ARMDBG V2 serial probe, bounded command and host-side named pose capture."""
import argparse
import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path

FIELDS = ["tick_ms", "debug_enabled", "ready", "active", "fault", "joint", "action",
          "shoulder_mdeg", "roll_mdeg", "gripper_mdeg", "elbow_mdeg",
          "pitch_mdeg", "shoulder_us", "elbow_us", "pitch_us", "roll_us",
          "gripper_us", "base_counts", "base_mdeg", "base_mrpm", "base_pwm",
          "uart_faults", "commands", "errors", "position_enabled_mask"]
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "output" / "arm-debug"


def parse_status(line):
    parts = line.strip().split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        return dict(zip(FIELDS, (int(part) for part in parts)))
    except ValueError:
        return None


def write_command(port, command):
    if "\n" in command or "\r" in command:
        raise ValueError("one command per call")
    port.write((command + "\n").encode("ascii"))
    port.flush()


def wait_reply(port, expected, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue
        if line.startswith("ERR "):
            raise RuntimeError(line)
        if line == expected:
            return
    raise TimeoutError(f"no reply: {expected}")


def collect_status(port, timeout=2):
    write_command(port, "ARM STATUS")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace")
        sample = parse_status(line)
        if sample is not None:
            return sample
        if line.startswith("ERR "):
            raise RuntimeError(line.strip())
    raise TimeoutError("no ARMDBG V2 25-channel status frame (old V1 firmware is incompatible)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="operation", required=True)
    sub.add_parser("probe", help="stop debug motion and verify UART acknowledgement/status")
    record = sub.add_parser("record", help="append a stopped commanded pose to a local JSONL file")
    record.add_argument("name")
    record.add_argument("--output", type=Path, default=DEFAULT_OUTPUT / "arm-poses.jsonl")
    watch = sub.add_parser("watch", help="capture status CSV without enabling motion")
    watch.add_argument("--seconds", type=float, default=5)
    watch.add_argument("--output", type=Path, default=DEFAULT_OUTPUT / "arm-status.csv")
    send = sub.add_parser("send", help="enable, send one bounded command with heartbeat, then stop")
    send.add_argument("command", help='e.g. "ARM POS 0 150 1000"')
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        parser.error("install pyserial first: python -m pip install pyserial")
    if args.operation == "watch" and not 0 < args.seconds <= 3600:
        parser.error("--seconds must be in (0, 3600]")
    duration = 0
    if args.operation == "send":
        tokens = args.command.split()
        try:
            if tokens[:2] in (["ARM", "POS"], ["ARM", "RATE"]):
                if len(tokens) != 5:
                    raise ValueError()
                duration = int(tokens[-1])
            else:
                raise ValueError()
            if not 1 <= duration <= 5000:
                raise ValueError()
        except ValueError:
            parser.error("use POS/RATE (1..5000 ms); one command only; old presets/neutral are unavailable")
    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1) as port:
        try:
            port.write(b"\n")  # resynchronize a partial/damaged line from a previous connection
            port.flush()
            write_command(port, "ARM DISABLE")
            wait_reply(port, "OK DISABLE")
            write_command(port, "ARM STREAM 0")
            wait_reply(port, "OK STREAM")
            if args.operation == "send":
                write_command(port, "ARM ENABLE")
                wait_reply(port, "OK ENABLE")
                write_command(port, args.command)
                wait_reply(port, "OK QUEUED")
                deadline = time.monotonic() + duration / 1000
                next_heartbeat = 0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_heartbeat:
                        write_command(port, "ARM HB")
                        next_heartbeat = now + 0.1
                    line = port.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("ERR "):
                        raise RuntimeError(line)
                write_command(port, "ARM STOP")
                wait_reply(port, "OK STOP")
                time.sleep(0.03)  # allow the 10 ms control owner to consume STOP
            if args.operation == "watch":
                args.output.parent.mkdir(parents=True, exist_ok=True)
                with args.output.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=FIELDS)
                    writer.writeheader()
                    deadline = time.monotonic() + args.seconds
                    while time.monotonic() < deadline:
                        writer.writerow(collect_status(port))
                        time.sleep(0.1)
                print(args.output.resolve())
            else:
                time.sleep(0.03)
                sample = collect_status(port)
                print(json.dumps(sample, ensure_ascii=False, indent=2))
                if args.operation == "record":
                    if not sample["ready"] or sample["active"] or sample["base_pwm"]:
                        raise RuntimeError("record requires software ready and stopped")
                    entry = {"name": args.name, "time": datetime.now(timezone.utc).isoformat(),
                             "protocol": "ARMDBG V2", "status": sample,
                             "position_servos": "commanded, not measured",
                             "base_reference": "relative to this MCU boot",
                             "base_position_limits_deg": [-180, 180],
                             "base_position_pickup_required": False,
                             "base_position_target_slew_dps": 90,
                             "servo_travel_deg": [300, 300, 270, 180, 180],
                             "mechanical_min_deg": [5, 23, 22, 0, 6],
                             "mechanical_max_deg": [150, 180, 270, 180, 45],
                             "jog_direction": [1, 1, -1, 1, 1],
                             "mechanical_limits": "user-confirmed 2026-10-28; old poses must not be replayed",
                             "position_enabled_mask": sample["position_enabled_mask"]}
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    with args.output.open("a", encoding="utf-8") as stream:
                        stream.write(json.dumps(entry, ensure_ascii=False) + "\n")
                    print(args.output.resolve())
        finally:
            write_command(port, "ARM DISABLE")


if __name__ == "__main__":
    main()
