import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class PWMSubscriber(Node):

    def __init__(self):
        super().__init__("pwm_subscriber")

        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]
        self._subscribers = []

        self.portName = serial.Serial("/dev/ttyACM0")
        for i, thruster in enumerate(self._thruster_ids):
            subscriber = self.create_subscription(
                Int16,
                f"thrusters/{thruster}/pwm",
                lambda msg, tn=i + 1: self.listener_callback(tn, msg),
                10,
            )
            self._subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to thrusters/{thruster}/pwm")

    def listener_callback(self, thruster_number, msg):
        # self.get_logger().info(f"Received PWM: {msg.data} for thruster {thruster_number}")

        pwm_value = msg.data
        command = f"{thruster_number} {pwm_value}"
        try:
            self.portName.write(command.encode())
            self.get_logger().info(
                f"Sent to thruster {thruster_number}: {pwm_value} pwm"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial port: {e}")


def main(args=None):
    rclpy.init(args=args)
    pwm_subscriber = PWMSubscriber()
    rclpy.spin(pwm_subscriber)
    pwm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
