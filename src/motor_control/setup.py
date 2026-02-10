from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/keyboard_control.launch.py',
            'launch/hs_motor_controller.launch.py',
            'launch/robot_state_publisher.launch.py',
            'launch/bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/hs_motor_config.yaml',
            'config/tf_config.yaml'
        ]),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Motor control package for differential drive robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hs_motor_controller = motor_control.hs_motor_controller:main',
            'map_relay = motor_control.map_relay:main',
            # Mock 節點 (模擬模式)
            'mock_motor_controller = motor_control.mock_motor_controller:main',
            'mock_lidar = motor_control.mock_lidar:main',
            'mock_imu = motor_control.mock_imu:main',
            # E-Stop 節點
            'e_stop_node = motor_control.e_stop_node:main',
        ],
    },
)
