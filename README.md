# SAUV-Autonomy

Autonomy stack

# Installation

1. Install VMWare Fusion (the free version, not the pro version).
2. Download the Ubuntu ISO from https://cdimage.ubuntu.com/jammy/daily-live/current/jammy-desktop-arm64.iso.
3. Open VMWare and drag the downloaded Debian ISO for installation.
4. Press continue, then finish, and name your instance (leaving the default name is fine).
5. Select "Try to install Ubuntu". Then click "Install Ubuntu" from the desktop.
6. Proceed with the installation leaving everything as the default values.
7. Shutdown the virtual machine (Virtual Machine > Shutdown).
8. Go to Virtual Machine > Settings > CD/DVD and select autodetect.
9. Still in settings, go to Startup Disk and select Hard Disk and then Restart.
10. When prompted for Ubuntu, press enter, and then login.
11. Run the following: `sudo apt-get update && sudo apt-get install open-vm-tools-desktop`
12. Shutdown the Virtual Machine, then restart it, and copy-pasting shortcuts should work.
13. Run the following in a Terminal: `sudo apt-get install openssh-server && sudo apt install net-tools && service ssh start`
14. Note the USER@HOST shown in the VM's Ubuntu Terminal. Then open a Terminal on your local computer (i.e. your Mac) and enter `ssh USER@HOST.local`.
15. If you connect successfully, now type `logout`, and then type `ssh-keygen -t ed25519` leaving all fields as default. Finally type `ssh-copy-id -i ~/.ssh/id_ed25519.pub USER@HOST.local`, once more replacing USER and HOST with your own credentials.
16. Install VSCode from https://code.visualstudio.com.
17. Install the VSCode Remote SSH extension.
18. Open a new VSCode window, press `CMD + Shift + P`, and type `Remote-SSH: Connect to Host` followed by `Enter`. Select `Add New SSH Host`. Enter `ssh USER@HOST.local` followed by `Enter`. Select your user's configuration file to update. Finally press connect.
19. Choose `Clone Repository` in the sidebar, select `Clone from GitHub`, and select the Stanford AUV Autonomy repository. Choose a folder to clone it in (something like `~/GitHub/`).
20. Open a Terminal in VSCode and run: `bash install.sh`
21. You're all set!

# Development

Make sure to use VSCode. Install all the extensions mentioned in the `.vscode/extensions.json` file.

Create a new VSCode Terminal, and run on each new terminal window:
```bash
export PYTHONPATH=${PYTHONPATH}:$PWD/src
```

# Building

First, make sure you have all the dependencies:
```bash
rosdep install -i --from-path src --rosdistro iron -y
```
To build the code, please run the following:
```bash
colcon build && source install/setup.bash
```

# Running

To run a ROS node, please run the following:
```bash
ros2 run MODULE_NAME NODE_NAME
```
For example:
```bash
ros2 run control thruster_manager
```

# Testing

To run tests, use the `unittest` module like so:
```bash
python3 -m unittest MODULE_NAME.tests.TEST_CLASS_NAME
```
For example:
```bash
python3 -m unittest control.tests.TestEKF
```
You can also run specific tests using:
```bash
python3 -m unittest MODULE_NAME.tests.TEST_CLASS_NAME.TEST_NAME
```
For example:
```bash
python3 -m unittest control.tests.TestEKF.test_initial_state
```