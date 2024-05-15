from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stanford RoboSub",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"checkpoint_manager = {package_name}.checkpoint_manager:main",
            f"thruster_manager = {package_name}.thruster_manager:main",
            f"pwm_subscriber = {package_name}.pwm_subscriber:main",
            f"gazebo_manager = {package_name}.gazebo_manager:main",
            f"state_estimator = {package_name}.state_estimator:main",
        ],
    },
)
