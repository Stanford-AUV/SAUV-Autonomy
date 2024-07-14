from setuptools import find_packages, setup
import os
from glob import glob

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stanford RoboSub",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"thruster_manager = {package_name}.thruster_manager:main",
            f"arduino = {package_name}.arduino:main",
            f"gazebo_manager = {package_name}.gazebo_manager:main",
            f"state_estimator = {package_name}.state_estimator:main",
            f"controller = {package_name}.controller:main",
            f"imu = {package_name}.imu:main",
            f"joystick = {package_name}.joystick:main",
            f"forward_publisher = {package_name}.forward_publisher:main",
            f"imu_noise_estimator = {package_name}.imu_noise_estimator:main"
        ],
    },
)
