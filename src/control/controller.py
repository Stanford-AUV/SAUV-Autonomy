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
        self.desired = self.state

    def set_state(self, msg) :
        self.state = msg

    def set_desired(self, msg) :
        self.desired = msg

    def update(self, dt) :
        error = self.desired - self.state
        derivative = (error - self.prev_error_) / dt

        # to account for n-dimensions, loop through all values in sub's state vector
        for i in range(self.dimension_) :
            # We wait to incorporate integral term to avoid windup
            if(error[i] <= self.p_start_i_[i][i]) :
                self.integral[i] += error[i] * dt
        
        output = (self.kP_ @ error) + (self.kD_ @ derivative) + (self.kI_ @ self.integral)

        # current error becomes previous error in the next loop
        self.prev_error = error

        return output.astype(float64)
    
    
class PoseSubscriber(Node) :

    def __init__(self, controller) :
        super().__init__('pose_subscriber')

        self.subscription_ = self.create_subscription(
            Float64MultiArray, 
            'state', 
            self.pose_callback,
            10 # Temporary queue size
        )

    def pose_callback(self, msg) :
        pose = np.array(msg.data)
        self.get_logger().info('Received pose: %s' % pose)
        controller.set_state(pose)


class DesiredSubscriber(Node) :
    
    def __init__(self) :
        super().__init__('desired_subscriber')

        self.subscription_ = self.create_subscription(
            Float64MultiArray,
            'desired_pos',
            self.desired_callback,
            10 # Temporary queue size
        )

    def desired_callback(self, msg) :
        desired = np.array(msg.data)
        self.get_logger().info('Received desired: %s' % desired)
        controller.set_desired(desired)


class PIDPublisher(Node) :

    def __init__(self) :
        super().__init__('output_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'desired_wrench',
            10
        )
        timer_period = .01 # seconds
        self.timer = self.create_timer(timer_period, self.publish_output)

        def publish_output(self) :
            msg = Float64MultiArray()
            msg.data = controller.update()
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing output: %s' % msg.data)


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
    pose_subscriber = PoseSubscriber()
    desired_subscriber = DesiredSubscriber()
    output_publisher = PIDPublisher()

    try :
        rclpy.spin(pose_subscriber)
        rclpy.spin(desired_subscriber)
        rclpy.spin(output_publisher)
    except KeyboardInterrupt :
        pass
    
    # Cleanup
    pose_subscriber.destroy_node()
    desired_subscriber.destroy_node()
    output_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__' :
    main()