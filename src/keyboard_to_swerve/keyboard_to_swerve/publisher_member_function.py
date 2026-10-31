import sys
import threading
import rclpy
from std_msgs.msg import Int32MultiArray

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


def get_key(settings):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = save_terminal_settings()

    rclpy.init()
    node = rclpy.create_node('simple_key_publisher')
    pub = node.create_publisher(Int32MultiArray, 'keyboard_inputs', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    print("q/w : 1번 스티어링 +1/-1")
    print("a/s : 2번 구동      +1/-1")
    print("e/r : 3번 스티어링 +1/-1")
    print("d/f : 4번 구동      +1/-1")
    print("CTRL+C to quit")

    try:
        while True:
            key = get_key(settings)

            cmd = [0, 0, 0, 0]
            published = False

            if key == 'q':
                cmd[0] = 1
                published = True
            elif key == 'w':
                cmd[0] = -1
                published = True
            elif key == 'a':
                cmd[1] = 1
                published = True
            elif key == 's':
                cmd[1] = -1
                published = True
            elif key == 'e':
                cmd[2] = 1
                published = True
            elif key == 'r':
                cmd[2] = -1
                published = True
            elif key == 'd':
                cmd[3] = 1
                published = True
            elif key == 'f':
                cmd[3] = -1
                published = True
            elif key == '\x03':  # CTRL+C
                break

            if published:
                msg = Int32MultiArray()
                msg.data = cmd
                pub.publish(msg)
                print(f"Published: {msg.data}")

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()