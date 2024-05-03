ign gazebo gazebo_maritime/worlds/auv_control.sdf -r \
& ign service -s /world/buoyant_tethys/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'reset: {all: true}' \
& ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=gazebo_bridge.yaml
