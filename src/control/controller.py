import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import threading
from std_msgs.msg import Float64
from std_srvs.srv import Empty

class PIDController(Node):

    def __init__(self, kP, kI, kD, p_start_i):
        super().__init__('pid_controller')
        self._kP = kP
        self._kI = kI
        self._kD = kD
        self._p_start_i = p_start_i
        self._integral = 0
        self._prev_error = 0
        self.running = False
        self.lock = threading.Lock()
        
        # Declare parameters and allow for runtime reconfiguration
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kP', self._kP),
                ('kI', self._kI),
                ('kD', self._kD),
                ('p_start_i', self._p_start_i)
            ]
        )

        # Subscribers and Publishers
        self.error_publisher = self.create_publisher(Float64, 'error', 10)
        self.output_publisher = self.create_publisher(Float64, 'controller_output', 10)
        
        # Services
        self.reset_service = self.create_service(Empty, 'reset_controller', self.handle_reset)

        # Timer setup
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.desired = 0
        self.state = 0

        # Add a parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'kP':
                self._kP = param.value
            elif param.name == 'kI':
                self._kI = param.value
            elif param.name == 'kD':
                self._kD = param.value
            elif param.name == 'p_start_i':
                self._p_start_i = param.value
        return SetParametersResult(successful=True)

    def update(self, desired, state, dt):
        error = desired - state
        derivative = (error - self._prev_error) / dt
        if abs(error) <= self._p_start_i:
            self._integral += error * dt

        output = self._kP * error + self._kI * self._integral + self._kD * derivative
        self._prev_error = error

        # Publishing error and output for debugging
        self.error_publisher.publish(Float64(data=error))
        self.output_publisher.publish(Float64(data=output))
        
        return output

    def timer_callback(self):
        with self.lock:
            self.update(self.desired, self.state, 0.1)

    def set_desired_state(self, desired):
        with self.lock:
            self.desired = desired

    def set_current_state(self, state):
        with self.lock:
            self.state = state

    def reset(self):
        with self.lock:
            self._integral = 0
            self._prev_error = 0

    def handle_reset(self, request, response):
        self.get_logger().info('Resetting controller settings.')
        self.reset()
        return response

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController(0.1, 0.01, 0.05, 0.5)
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
