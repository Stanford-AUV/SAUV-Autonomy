import rospy


class Controller :
    """
    A class representing a PID controller
    """

    def __init__(self, kP, kI, kD, p_start_i) :
        """
        Initialization for Controller class

        kP - error gain
        kI - integral gain
        kD - derivative gain
        p_start_i - error at which to start integral term
        """
        self._kP = kP
        self._kI = kI
        self._kD = kD
        self._p_start_i = p_start_i
        self._integral = 0
        self._prev_error = 0
        self.desired = 0

    def set_desired(self, desired) :
        """
        Set the desired value for the controller to reach
        """
        self.desired = desired
    
    def update(self, state) :
        """
        Updates the controller and returns new output power

        state - robot's current state
        """
        error = self.desired - state
        derivative = error - self._prev_error
        # We wait to incorporate integral term to avoid windup
        if(error <= self._p_start_i) :
            self._integral += error
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