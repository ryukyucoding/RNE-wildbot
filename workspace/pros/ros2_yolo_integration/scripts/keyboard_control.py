"""
Keyboard teleop for pros_twin Unity virtual environment via roslibpy.

Topics published:
  /car_C_rear_wheel  (std_msgs/msg/Float32MultiArray) — [rear_left, rear_right]
  /car_C_front_wheel (std_msgs/msg/Float32MultiArray) — [front_left, front_right]

Keys:
  w — forward       s — backward
  a — turn left     d — turn right
  e — rotate CCW    r — rotate CW
  q — shift left    f — shift right
  z — stop          Ctrl+C — quit
"""

import os
import sys
import tty
import termios
import threading
import roslibpy

SPEED = 300.0
ROTATE = 300.0

ACTION_MAPPINGS = {
    "w": [ SPEED,  SPEED,  SPEED,  SPEED],   # forward
    "s": [-SPEED, -SPEED, -SPEED, -SPEED],   # backward
    "a": [ ROTATE, ROTATE * 1.2, ROTATE, ROTATE * 1.2],  # left front
    "d": [ ROTATE * 1.2, ROTATE, ROTATE * 1.2, ROTATE],  # right front
    "e": [-ROTATE,  ROTATE, -ROTATE,  ROTATE],  # CCW rotation
    "r": [ ROTATE, -ROTATE,  ROTATE, -ROTATE],  # CW rotation
    "q": [ ROTATE, -ROTATE, -ROTATE,  ROTATE],  # shift left
    "f": [-ROTATE,  ROTATE,  ROTATE, -ROTATE],  # shift right
    "z": [0.0, 0.0, 0.0, 0.0],                 # stop
}

MSG_TYPE = "std_msgs/msg/Float32MultiArray"


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def publish_wheels(rear_pub, front_pub, velocities):
    rear_l, rear_r, front_l, front_r = velocities
    rear_pub.publish(roslibpy.Message({"data": [rear_l, rear_r]}))
    front_pub.publish(roslibpy.Message({"data": [front_l, front_r]}))


def main():
    ros_host = os.environ.get("ROS_HOST", "localhost")
    ros_port = int(os.environ.get("ROS_PORT", 9090))

    print(f"Connecting to rosbridge at ws://{ros_host}:{ros_port} ...")
    client = roslibpy.Ros(host=ros_host, port=ros_port)
    client.run()

    if not client.is_connected:
        print("ERROR: Could not connect to rosbridge. Make sure pros_app rosbridge_server.sh is running.")
        sys.exit(1)

    print("Connected.")

    rear_pub  = roslibpy.Topic(client, "/car_C_rear_wheel",  MSG_TYPE)
    front_pub = roslibpy.Topic(client, "/car_C_front_wheel", MSG_TYPE)
    rear_pub.advertise()
    front_pub.advertise()

    print(
        "\n--- Keyboard Control ---\n"
        "  w/s  : forward / backward\n"
        "  a/d  : turn left / right\n"
        "  e/r  : rotate CCW / CW\n"
        "  q/f  : shift left / right\n"
        "  z    : stop\n"
        "  Ctrl+C : quit\n"
        "------------------------\n"
    )

    try:
        while client.is_connected:
            key = get_key()
            if key == "\x03":   # Ctrl+C
                break
            if key in ACTION_MAPPINGS:
                publish_wheels(rear_pub, front_pub, ACTION_MAPPINGS[key])
                label = {
                    "w": "FORWARD", "s": "BACKWARD",
                    "a": "LEFT",    "d": "RIGHT",
                    "e": "ROT CCW", "r": "ROT CW",
                    "q": "SHIFT L", "f": "SHIFT R",
                    "z": "STOP",
                }.get(key, key)
                print(f"\r[{label}]   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping wheels and disconnecting...")
        publish_wheels(rear_pub, front_pub, [0.0, 0.0, 0.0, 0.0])
        rear_pub.unadvertise()
        front_pub.unadvertise()
        client.terminate()
        print("Done.")


if __name__ == "__main__":
    main()
