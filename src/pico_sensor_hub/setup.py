from setuptools import find_packages, setup

package_name = 'pico_sensor_hub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pico_sensor_hub.launch.py',
            'launch/mock_pico_sensor_hub.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/pico_sensor_hub.yaml',
        ]),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Pico 感測器集線板橋接：8 通道超音波與 INA226 電源資料進 ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pico_sensor_node = pico_sensor_hub.pico_sensor_node:main',
            # /pico/voltage + /pico/current → sensor_msgs/BatteryState
            'battery_state_node = pico_sensor_hub.battery_state_node:main',
            # Mock 節點 (模擬模式)
            'mock_pico_sensor = pico_sensor_hub.mock_pico_sensor:main',
        ],
    },
)
