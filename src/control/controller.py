import rclpy
from rclpy.node import Node
from rclpy_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
import numpy as np
import threading



def get_pose_from_state(state) :
    """
    Prune state.msg to just the position and quaternion.

    :param state: Float64MultiArray representing position, quaternion, velocity, 
                  & acceleration.
    :return pose: x, y, z, roll, pitch, yaw.
    """
    return np.array([
        state.position.x,
        state.position.y,
        state.position.z,
        state.orientation.x,
        state.orientation.y,
        state.orientation.z
    ])

def clamp(value, limits) :
    """
    Clamp a value to bounds

    :param value: The value to be clamped.
    :param limits: List containing the bounds
    """
    return # TODO: finish in the future


class Controller(Node) :
    def __init__(self, p_values, i_values, d_values, start_i_values) :
        """
        Initialize Controller object with PID parameters for position and 
        orientation.

        :param p_values: Diagonal matrix of P gains for the controller 
            [pX, pY, pZ, pRoll, pPitch, pYaw].
        :param i_values: Diagonal matrix of I gains for the controller 
            [iX, iY, iZ, iRoll, iPitch, iYaw].
        :param d_values: Diagonal matrix of D gains for the controller 
            [dX, dY, dZ, dRoll, dPitch, dYaw].
        :param start_i_values: Diagonal matrix of values to start integral for the 
            controller [startX, startY, startZ, startRoll, startPitch, startYaw].
        """
        super().__init__('controller')

        self.kP_ = p_values
        self.kI_ = i_values
        self.kD_ = d_values
        self.start_I_ = start_i_values

        self.dim_ = 6   # 6 DOF

        # Initialize mutables; everything is 0 at t_0
        self.integral = np.zeros(self.dim_)
        self.derivative = np.zeros(self.dim_)
        self.desired = np.zeros(self.dim_)
        self.prev_error = np.zeros(self.dim_)
        self.pose = np.zeros(self.dim_)

        # Initialize current state subscriber
        self.state_subscription_ = self.create_subscription(
            Float64MultiArray, 
            'state', 
            self.pose_callback, 
            10
        )

        # Initialize desired state subscriber
        self.desired_subscription_ = self.create_subscription(
            Float64MultiArray, 
            'desired_pos', 
            self.desired_callback, 
            10
        )

        # Initialize force/torque output publisher
        self.output_publisher_ = self.create_publisher(
            Float64MultiArray, 
            'desired_wrench', 
            10
        )
        self.timer_period_ = .01   # seconds (100 Hz or 10 ms update cycle)
        self.timer = self.create_timer(self.timer_period_, self.update)

        # Services
        self.reset_service = self.create_service(
            Empty, 
            'reset_controller', 
            self.reset
        )

        # Create mutex to prevent race conditions
        self.running = False
        self.lock = threading.Lock()

    def pose_callback(self, msg) :
        """Get our current pose from a topic."""
        with self.lock :
            self.pose = np.array(get_pose_from_state(msg.data))
            self.get_logger().info('Received pose: %s' % self.pose)

    def desired_callback(self, msg) :
        """Get the desired pose from a topic."""
        with self.lock :
            self.desired = np.array(get_pose_from_state(msg.data))
            self.get_logger().info('Received desired pose: %s' % self.desired)

    def update(self) :
        """Update the controller with the current state and publish to a topic"""
        error = self.desired - self.pose
        self.get_logger().info('Received error: %s' % error)
        self.derivative = error - self.prev_error

        # Pose is represented as a vector, so we loop through all elements to 
        # incorporate integral term
        for i in range(self.dim_) :
            # Delay integral term to avoid integral wind-up
            if error[i] <= self.start_I_[i][i] :
                self.integral[i] += error[i]

        # Output is a linear combination of error, derivative, and integral vectors
        output = self.kP_ @ error + self.kI_ @ self.integral + self.kD_ @ self.derivative

        # Publish a list of control outputs:
        # [force_x, force_y, force_z, torque_roll, torque_pitch, torque_yaw]
        msg = Float64MultiArray(data = output)
        self.output_publisher_.publish(msg)
        self.get_logger().info('Publishing output: %s' % msg.data)

        # Current error becomes previous error in the next time step
        self.prev_error = error

    def reset(self, request, response) :
        """
        Reset the controller settings.

        :param request: The request message sent by the service (if any).
        :param response: The response message sent to the service (if any).
        :return response: The response, indicating that reset is successful.
        """
        self.get_logger().info('Resetting controler settings.')
        with self.lock :
            self.integral = 0
            self.derivative = 0
            self.prev_error = 0
            return response
        
def main(args=none) :
    rclpy.init(args=args)

    # Initialize controller gains
    #                      x, y, z, r, p, y
    kP = np.diag(np.array([1, 1, 1, 1, 1, 1]))
    kD = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    kI = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    start_I = np.diag(np.array([0, 0, 0, 0, 0, 0]))

    controller_node = Controller(kP, kD, kI, start_I)

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    controller_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__' :
    main()