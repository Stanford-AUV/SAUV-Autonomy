from setuptools import find_packages, setup

package_name = 'perception'

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
            'perception_node = perception.perception_node:main',
            'debug_node = perception.debug_node:main'
    maintainer='Stanford Robosub',
    maintainer_email='selenasun02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"oakd = {package_name}.oakd:main",
            'perception_node = perception.perception_node:main',
            'debug_node = perception.debug_node:main'
        ],
    },
)
