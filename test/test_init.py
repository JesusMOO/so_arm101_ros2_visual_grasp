#确认初始设备是不是没有问题（舵机+摄像头）
import cv2
from st3215 import ST3215


PORT = "/dev/ttyACM0"
CAMERA_INDEX = 4


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX,cv2.CAP_V4L2)
    if cap.isOpened():
        print("camera cap_show open success")
    else:
        print("camera cap_show open failed")
    cap.release()

    servo = ST3215(PORT)
    servos = servo.ListServos()
    servo.portHandler.closePort()

    if len(servos) == 6:
        print(f"servos' ids: {servos}")
        print("st3215 scan 6 servos success")
    else:
        print(f"st3215 scan failed, found {len(servos)} servos: {servos}")


if __name__ == "__main__":
    main()
