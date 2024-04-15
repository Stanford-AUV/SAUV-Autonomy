import rospy

class Controller :

    def __init__(self, kP, kI, kD, p_start_i) :
        self._kP = kP
        self._kI = kI
        self._kD = kD
        self._p_start_i = p_start_i
        self._integral = 0
        self._prev_error = 0
        self._desired = 0

    def set_desired(self, desired) :
        self._desired = desired
    
    #TODO: Create threading process and loop the update method
    def update(self, feedback) :
        error = self._desired - feedback
        derivative = error - self._prev_error
        if(error <= self._p_start_i) :
            self._integral += error
        output = error * self._kP + self._integral * self._kI + derivative * self._kD
        self._prev_error = error
        return output
    
    def reset(self) :
        self._integral = 0
        self._prev_error = 0