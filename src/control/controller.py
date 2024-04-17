import rclpy
import threading


class PIDController :

    def __init__(self, kP, kI, kD, p_start_i) :
        """
        Initialization for Controller class

        :param kP: Error gain
        :param kI: Integral gain
        :param kD: Derivative gain
        :param p_start_i: Error at which to start integral term
        """
        self._kP = kP
        self._kI = kI
        self._kD = kD
        self._p_start_i = p_start_i
        self._integral = 0
        self._prev_error = 0
        self.running = False
        self.lock = threading.Lock()

    
    def update(self, desired, state, dt) :
        """
        Updates the controller and returns new output power

        :param desired: The desired state of the robot
        :param state: Robot's current state
        :param dt: Time step from last control loop, in seconds
        :return: Control signal
        """
        error = desired - state
        derivative = (error - self._prev_error) / dt
        # We wait to incorporate integral term to avoid windup
        if(error <= self._p_start_i) :
            self._integral += error * dt
        output = error * self._kP + self._integral * self._kI + derivative * self._kD
        # Current error becomes the previous in the next loop
        self._prev_error = error
        return output

    def reset(self) :
        """
        Resets the controller by setting the integral and previous error to 0
        """
        self._integral = 0
        self._prev_error = 0

    def start(self, feedback_source) :
        """
        Starts the rospy node and PID controller thread
        """
        rclpy.spin()
        self.running = True

    def stop(self):
        """
        Stops the rospy node and shuts down the controller thread
        """
        self.running = False