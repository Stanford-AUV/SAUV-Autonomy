from setuptools import find_packages, setup

package_name = 'ControlTest'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomasd',
    maintainer_email='thomasxdeng@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pose_talker = ControlTest.pose_publisher:main',
        'desired_talker = ControlTest.desired_publisher:main', 
        'Controller = ControlTest.controller:main'
        ],
    },
)