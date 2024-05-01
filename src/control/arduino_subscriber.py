import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from functools import partial
from serial.tools import list_ports

class PWMSubscriber(Node):

    def __init__(self):
        super().__init__("pwm_subscriber")

        self._thruster_ids = [f"thruster{i}" for i in range(1, 9)]
        self._subscribers = []

        port = self.find_serial_port()
        if port:
            self.portName = serial.Serial(port)
        else:
            raise RuntimeError("No serial port found")

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
        try:
            self.portName.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
    
    def find_serial_port(self):
        # Get a list of all available serial ports
        ports = list_ports.comports()
        for port in ports:
            if "USB" in port.description:  # Check if it's a USB serial port
                return port.device
        return None

def main(args=None):
    rclpy.init(args=args)
    pwm_subscriber = PWMSubscriber()
    rclpy.spin(pwm_subscriber)
    pwm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
