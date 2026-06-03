from st3215 import ST3215


PORT = "/dev/ttyACM0"
MIN_RAW = 0
MAX_RAW = 4095


def clamp_raw(raw):
    return max(MIN_RAW, min(MAX_RAW, raw))


def parse_command(text):
    text = text.strip().lower()
    if text == "q":
        return None
    if not text.startswith("m") or ":" not in text:
        raise ValueError("format should be m1:2048")

    servo_text, raw_text = text[1:].split(":", 1)
    servo_id = int(servo_text)
    raw = clamp_raw(int(raw_text))
    return servo_id, raw


def main():
    servo = ST3215(PORT)
    print(f"opened {PORT}")
    print("input m1:2048 to move servo ID 1 to raw 2048")
    print("input q to quit")

    try:
        while True:
            text = input("> ")
            if text.strip().lower() == "q":
                break

            try:
                servo_id, raw = parse_command(text)
            except ValueError as exc:
                print(exc)
                continue

            ok = servo.MoveTo(servo_id, raw)
            if ok:
                print(f"m{servo_id} move to {raw} success")
            else:
                print(f"m{servo_id} move to {raw} failed")
    finally:
        servo.portHandler.closePort()
        print("closed port")


if __name__ == "__main__":
    main()
