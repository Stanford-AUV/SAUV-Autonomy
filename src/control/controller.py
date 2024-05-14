import rclpy
from rclpy.node import Node
from rclpy_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
import numpy as np
import threading

# Prune state.msg to just the position and quaternion (i.e. 'pose')
def prune_state(state) :
    pose = np.array([
        state.position.x,
        state.position.y,
        state.position.z,
        state.orientation.x, # roll
        state.orientation.y, # pitch
        state.orientation.z  # yaw
    ])

def clamp(value, limits) :
    lower, upper = limits
    if value is None:
        return None
    elif(upper is not None) and (value > upper) :
        return upper
    elif(lower is not None) and (value < lower) :
        return lower
    return value


class PIDController(Node) :
    
    def __init__(self, kP, kD, kI, p_start_i) :
        # Gains are a diagonal matrix
        self.kP_ = kP
        self.kD_ = kD
        self.kI_ = kI
        self.p_start_i = p_start_i

        self.dimension_ = 6 # dim(matrix)

        # Initialize controller iterables
        # At first time step everything is 0
        self.integral = np.zeros(self.dimension_)
        self.prev_error = np.zeros(self.dimension_)

        # Declare current and desired pose variables
        self.pose = np.zeros(self.dimension_)
        self.desired = np.zeros(self.dimension_)

        # Initialize state subscriber
        super().__init__('state_subscriber')
        self.subscription_ = self.create_subscription(
            Float64MultiArray, 
            'state', 
            self.pose_callback,
            10 # Temporary queue size
        )

        # Initialize desired state subscriber
        super().__init__('desired_subscriber')
        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'desired_pos',
            self.desired_callback,
            10 # Temporary queue size
        )

        # Initialize wrench publisher
        super().__init__('output_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'desired_wrench',
            10
        )
        self.timer_period_ = .01 # seconds (10ms / 100Hz)
        self.timer = self.create_timer(self.timer_period_, self.publish_output)

        # Create mutex
        self.running = False
        self.lock = threading.Lock()
        
        # Declare parameters and allow for runtime reconfiguration
        self.declare_parameters(
            namespace = '', 
            parameters = [
                ('kP', self.kP_),
                ('kD', self.kD_),
                ('kI', self.kI_),
                ('p_start_i', self.p_start_i)
            ]
        )

        # Services
        self.reset_service = self.create_service(Empty, 'reset_controller', self.handle_reset)

        # Add a parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params) :
        for param in params :
            if param.name == 'kP' :
                self.kP_ = param.value
            if param.name == 'kD' :
                self.kD_ = param.value
            if param.name == 'kI' :
                self.kI_ = param.value
            if param.name == 'p_start_i' :
                self.p_start_i_ = param.value

    def pose_callback(self, msg) :
        with self.lock :
            self.pose = np.array(prune_state(msg.data))
            self.get_logger().info('Received pose: %s' % self.pose)
        
    def desired_callback(self, msg) :
        with self.lock :
            self.desired = np.array(prune_state(msg.data))
            self.get_logger().info('Received desired pose: %s' % self.desired)

    def update_wrench(self) :
        error = self.desired - self.pose
        derivative = error - self.prev_error

        # Pose represented as a vector, so we loop through elements
        for i in range(self.dimension_) :
            # Delay integral term to avoid integral windup
            if(error[i] <= self.p_start_i[i][i]) :
                self.integral[i] += error[i]

        # Wrench is a linear combination of error, derivative, and integral vectors
        wrench = (self.kP_ @ error) + (self.kD_ @ derivative) + (self.kI_ @ self.integral) 

        # Current error becomes previous error in the next time step
        self.prev_error = error
        
        # Publish the wrench
        msg = Float64MultiArray(data = wrench)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing output %s' % msg.data)

    def reset(self) :
        with self.lock :
            self.integral = 0
            self.prev_error = 0

    def handle_reset(self, request, response) :
        self.get_logger().info('Resetting controller settings.')
        self.reset()
        return response


def main(args=none) :
    rclpy.init(args=args)

    # Initialize PID gains
    #                      x, y, z, r, p, y
    kP = np.diag(np.array([1, 1, 1, 1, 1, 1]))
    kD = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    kI = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    p_start_i = np.diag(np.array([0, 0, 0, 0, 0, 0]))

    controller_node = PIDController(kP, kD, kI, p_start_i)

    try :
        rclpy.spin(controller_node)
    except KeyboardInterrupt :
        pass
    
    # Cleanup
    controller_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__' :
    main()