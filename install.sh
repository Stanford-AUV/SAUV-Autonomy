sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=${PYTHONPATH}:$PWD/src" >> ~/.bashrc
source ~/.bashrc
printenv | grep -i ROS
sudo apt-get install -y ros-${ROS_DISTRO}-ros-gz
sudo apt-get install -y ros-${ROS_DISTRO}-ros-ign-bridge
sudo apt-get install -y ros-${ROS_DISTRO}-teleop-twist-keyboard
sudo apt install -y ros-${ROS_DISTRO}-nmea-msgs
sudo apt install -y ros-${ROS_DISTRO}-mavros-msgs
sudo apt install -y ros-${ROS_DISTRO}-tf-transformations
python3 -m pip install -r requirements.txt
sudo rosdep init
rosdep update
