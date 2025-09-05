from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/keyboard_control.launch.py', 
            'launch/esp32_motor_controller.launch.py',
            'launch/full_system.launch.py',
            'launch/robot_state_publisher.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/motor_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_send_to_esp32 = motor_control.uart_send_to_esp32:main'
        ],
    },
)
