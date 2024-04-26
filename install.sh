sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install python3-pip
sudo apt install ros-iron-desktop
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=${PYTHONPATH}:$PWD/src" >> ~/.bashrc
source ~/.bashrc
printenv | grep -i ROS
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
python3 -m pip install -r requirements.txt
sudo rosdep init
rosdep update