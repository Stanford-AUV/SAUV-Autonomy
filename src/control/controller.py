import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np


class PIDController :

    def __init__(self, kP, kD, kI, p_start_i) :
        self.kP_ = kP
        self.kD_ = kD
        self.kI_ = kI

        self.p_start_i_ = p_start_i

        self.dimension_ = kP.ndim()
        self.integral = np.zeros(self.dimension_)
        self.prev_error = np.zeros(self.dimension_)

    def reset(self) :
        self.integral = np.zeros(self.dimension_)
        self.prev_error = np.zeros(self.dimension_)

    def set_state(self, msg) :
        self.state = msg

    def update(self, desired, dt) :
        error = desired - self.state
        derivative = (error - self.prev_error_) / dt

        # to account for n-dimensions, loop through all values in sub's state vector
        for i in range(self.dimension_) :
            # We wait to incorporate integral term to avoid windup
            if(error[i] <= self.p_start_i_[i][i]) :
                self.integral[i] += error[i] * dt
        
        output = (self.kP_ @ error) + (self.kD_ @ derivative) + (self.kI_ @ self.integral)

        # current error becomes previous error in the next loop
        self.prev_error = error

        return output
    
    
class PIDSubscriber(Node) :

    def __init__(self, controller) :
        super().__init__('pose_subscriber')

        self.subscription_ = self.create_subscription(
            np.array(), 
            'pose', 
            self.pid_callback,
            10
        )

    def pid_callback(self, msg) :
        pose = np.array(msg.data)
        self.get_logger().info('Received pose: %s' % pose)
        controller.set_state(pose)

    

class PIDPublisher(Node) :

    def __init__(self) :
        super().__init__('output_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'controller_output',
            10
        )

        def publish_output(self, output) :
            msg = Float64MultiArray()
            msg.data = output
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing output: %s' % output)


def main(args=None) :
    rclpy.init(args=args)

    # Initialize PID gains
    kP = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    kD = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    kI = np.diag(np.array([0, 0, 0, 0, 0, 0]))
    p_start_i = np.diag(np.array([0, 0, 0, 0, 0, 0]))

    # Create global controller object
    global controller 
    controller = PIDController(kP, kD, kI, p_start_i)

    # Create pose subscriber and output publisher
    pose_subscriber = PIDSubscriber()
    output_publisher = PIDPublisher()

    try :
        rclpy.spin(pose_subscriber)
        rclpy.spin(output_publisher)
    except KeyboardInterrupt :
        pass

    pose_subscriber.destroy_node()
    output_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__' :
    main()