# SAUV-Autonomy

Autonomy stack

# Installation

Make sure ROS 2 Iron is fully installed.
Also, from inside the root folder of this repository, run the following in every new Terminal:
```bash
export PYTHONPATH=${PYTHONPATH}:$PWD/src
```

# Development

Make sure to use VSCode. Install all the extensions mentioned in the `.vscode/extensions.json` file.

# Building

To build the code, please run the following:
```bash
colcon build && . install/setup.zsh
```

# Running

To run a ROS node, please run the following:
```bash
ros2 run MODULE_NAME NODE_NAME
```

# Testing

To run tests, use the `unittest` module like so:
```bash
python -m unittest MODULE_NAME.tests.TEST_CLASS_NAME
```
For example:
```bash
python -m unittest control.tests.TestEKF
```
You can also run specific tests using:
```bash
python -m unittest MODULE_NAME.tests.TEST_CLASS_NAME.TEST_NAME
```
For example:
```bash
python -m unittest control.tests.TestEKF.test_initial_state
```