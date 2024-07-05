Header header

time ros_time # assuming this is /utctime
time imu_time # for /imu/imu_time

# bool triggered_dvl # NOT SURE HOW TO READ FROM IMU

geometry_msgs/Vector3 orientation # for /filter 
geometry_msgs/QuaternionStamped orientation_quat # for /filter/quaternion

geometry_msgs/TwistStamped twist # for /filter/twist which is NOT in quaternions!
geometry_msgs/QuaternionStamped rate_of_turn_quat # for imu/dq

geometry_msgs/Vector3 linear_acceleration # /imu/acceleration
geometry_msgs/Vector3 free_acceleration # /filter/free_acceleration