"""
Suggested structure(?):

Orientation control:
1. Initialize ROS nodes (yaw, pitch, roll)
2. Pass those nodes into respective PID controllers (yaw, pitch, roll)
3. Start threads to control 3 rotational axis
    - This will allow us to pass in any desired value on any axis and output appropriate thrust

Localization:
1. Initialize ROS node for the IMU and DVL
2. Double integrate on IMU accelerometer and integrate on the DVL for position
3. Pass into EKF to stabilize localization estimate
4. EKF will output a state vector including position, velocity, and acceleration

Position control: 
1. Figure out how thrusters influence robot movement ()
"""