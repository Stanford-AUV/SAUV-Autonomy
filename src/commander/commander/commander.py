import sys
import threading

from msgs.msg import Cmd
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Missing init message!

CTRL-C to quit
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('commander')

    pub = node.create_publisher(Cmd, 'command', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    CmdMsg = Cmd()

    try:
        while True:
            key = getKey(settings)
            if (key == "x" or key == "X"):
                CmdMsg.enable_thrusters = False
                CmdMsg.enable_controller = False
                CmdMsg.enable_estimator = False
            elif (key == "g"):
                CmdMsg.enable_thrusters = True
            else:
                if key == '\x03':
                    break

            pub.publish(CmdMsg)

    except Exception as e:
        print(e)

    finally:
        CmdMsg.enable_thrusters = False
        CmdMsg.enable_controller = False
        CmdMsg.enable_estimator = False
        pub.publish(CmdMsg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
