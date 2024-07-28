from setuptools import find_packages, setup

package_name = "firmware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/" + package_name, [package_name + "/ms5837.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mk",
    maintainer_email="miyukojima2010@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_talker = firmware.depth_publisher:main",
            "sync_filter = firmware.sync_filter:main",
            "sync_imu = firmware.sync_imu:main",
        ],
    },
)
