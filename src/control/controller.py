import rclpy
import threading
import numpy as np


# class PIDController :

#     def __init__(self, kP, kI, kD, p_start_i) :
#         """
#         Initialization for PIDController class

#         :param kP: Error gain
#         :param kI: Integral gain
#         :param kD: Derivative gain
#         :param p_start_i: Error at which to start integral term
#         """
#         self._kP = kP
#         self._kI = kI
#         self._kD = kD
#         self._p_start_i = p_start_i
#         self._integral = 0
#         self._prev_error = 0
#         self.running = False
#         self.lock = threading.Lock()

    
#     def update(self, desired, state, dt) :
#         """
#         Updates the controller and returns new output power

#         :param desired: The desired state of the robot
#         :param state: Robot's current state
#         :param dt: Time step from last control loop, in seconds
#         :return: Control signal
#         """
#         error = desired - state
#         derivative = (error - self._prev_error) / dt
#         # We wait to incorporate integral term to avoid windup
#         if(error <= self._p_start_i) :
#             self._integral += error * dt
#         output = error * self._kP + self._integral * self._kI + derivative * self._kD
#         # Current error becomes the previous in the next loop
#         self._prev_error = error
#         return output

#     def reset(self) :
#         """
#         Resets the controller by setting the integral and previous error to 0
#         """
#         self._integral = 0
#         self._prev_error = 0

#     def start(self, feedback_source) :
#         """
#         Starts the rospy node and PID controller thread
#         """
#         rclpy.spin()
#         self.running = True

#     def stop(self):
#         """
#         Stops the rospy node and shuts down the controller thread
#         """
#         self.running = False


class nPIDController :
    """
    PID Controller class that works in n-dimensions
    """

    def __init__(self, kP, kI, kD, p_start_i) :
        """
        Initialization for nPIDController class

        :param kP: Error gains; np matrix
        :param kI: Integral gains; np matrix
        :param kD: Derivative gains; np matrix
        :param p_start_i: Error at which to start integral term; np matrix
        """
        self.kP_ = kP
        self.kI_ = kI
        self.kD_ = kD
        self.p_start_i_ = p_start_i
        self.dimension_ = kP.ndim()
        self.integral_ = np.zeros(self.dimension_)
        self.prev_error_ = np.zeros(self.dimension_)
        self.running = False
        self.lock = threading.Lock()

    def reset(self) :
        """
        Resets the controller by setting the integral and previous error arrays to 0
        """
        self.integral_ = np.zeros(self.dimension_)
        self.integral_ = np.zeros(self.dimension_)

    def update(self, desired, state, dt) :
        """
        Updates the controller and returns new output power

        :param desired: The desired state of the robot; np array
        :param state: Robot's current state; np array
        :param dt: Time step from last control loop, in seconds
        :return: Control signal; np array
        """
        error = desired - state
        derivative = (error - self._prev_error) / dt
        # to account for n-dimensions, loop through all values in sub's state
        for i in range(self.dimension_) :
            # We wait to incorporate integral term to avoid windup
            if(error[i] <= self._p_start_i[i][i]) :
                self._integral[i] += error[i] * dt
        output = self.kP_ @ error  + self.kI_ @ self.integral + self.kD_ @ derivative
        # Current error becomes the previous in the next loop
        self._prev_error = error
        return output