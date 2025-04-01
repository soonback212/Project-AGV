from setuptools import find_packages, setup

package_name = 'slam_control'

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
    maintainer='jiwon',
    maintainer_email='jiwon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_parser_node = slam_control.imu_parser_node:main',
            'motor_serial_node = slam_control.motor_serial_node:main',
        ],
    },
)
