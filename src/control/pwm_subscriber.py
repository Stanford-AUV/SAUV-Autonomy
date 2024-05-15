import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from functools import partial

class PWMSubscriber(Node):

    def __init__(self):
        super().__init__("pwm_subscriber")

        self._thruster_ids = [f"thruster{i}" for i in range(1, 9)]
        self._subscribers = []

        self.portName = serial.Serial("/dev/ttyACM0") # TODO REPLACE
        for i, thruster in enumerate(self._thruster_ids):
            subscriber = self.create_subscription(
                Int16, 
                f"thrusters/{thruster}/pwm",
                partial(self.pwm_callback, thruster_number=i+1),
                10
            )
            self._subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to {thruster} PWM")

    def pwm_callback(self, thruster_number, msg):
        self.get_logger().info(f'Received PWM: {msg.data} for thruster {thruster_number}')

        pwm_value = msg
        command = f"{thruster_number} {pwm_value}"
        self.portName.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    pwm_subscriber = PWMSubscriber()
    rclpy.spin(pwm_subscriber)
    pwm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
