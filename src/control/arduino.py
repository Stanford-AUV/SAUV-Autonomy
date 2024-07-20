import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from msgs.msg import Cmd

class Arduino(Node):

    def __init__(self):
        super().__init__("arduino")

        self._zero_thrust = 1500
        self._allow_thrust = True # thrust enable flag
        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]
        self._subscribers = []

        try:
            self.portName = serial.Serial("/dev/ttyUSB_teensy", baudrate=9600, timeout=1)
            self.get_logger().info(f"Serial port /dev/ttyUSB_teensy opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        for i, thruster in enumerate(self._thruster_ids):
            subscriber = self.create_subscription(
                Int16,
                f"thrusters/{thruster}/pwm",
                lambda msg, tn=i+2: self.listener_callback(tn, msg),
                10
            )
            self._subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to thrusters/{thruster}/pwm")

        self.cmd_sub = self.create_subscription(Cmd, "command", 10, self.cmd_callback)

    def listener_callback(self, thruster_number, msg):
        # self.get_logger().info(f"Received PWM: {msg.data} for thruster {thruster_number}")
        pwm_value = self._zero_thrust
        if(False == self._allow_thrust): # nullify all thurst commands if flag is not enabled
            self.get_logger().info(f'[MOTORS DISABLED] - Received {msg.data} for thruster {thruster_number}')
        else:
            pwm_value = msg.data 

        command = f'{thruster_number} {pwm_value}\n'
        try:
            self.portName.write(command.encode())
            # self.get_logger().info(f'Sent to thruster {thruster_number}: {pwm_value} pwm')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {e}')

    def cmd_callback(self, msg):
        if msg.enable_thrust == True:
            self._allow_thrust = True
        elif msg.enable_thrust == False: # if false, prevent commands and set all thrusters to zero
            self._allow_thrust = False
            for i in enumerate(self._thruster_ids):
                command = f'{i+2} {self._zero_thrust}\n'
                try:
                    self.portName.write(command.encode())
                    # self.get_logger().info(f'Sent to thruster {thruster_number}: {pwm_value} pwm')
                except serial.SerialException as e:
                    self.get_logger().error(f'Failed to write to serial port: {e}')
                 

def main(args=None):
    rclpy.init(args=args)
    arduino = Arduino()
    rclpy.spin(arduino)
    arduino.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
