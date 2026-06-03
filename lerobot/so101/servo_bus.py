from st3215 import ST3215

#封装 ST3215 舵机通信
class ServoBus:
    def __init__(self, port):
        self.port = port
        self.servo = ST3215(port)

    def scan_servos(self):
        return self.servo.ListServos()

    def read_position(self, servo_id):
        return self.servo.ReadPosition(servo_id)

    def move_to(self, servo_id, raw, speed=2400, acc=50, wait=False):
        return self.servo.MoveTo(servo_id, raw, speed=speed, acc=acc, wait=wait)

    def stop(self, servo_id):
        return self.servo.StopServo(servo_id)

    def close(self):
        self.servo.portHandler.closePort()
