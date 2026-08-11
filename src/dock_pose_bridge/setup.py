from setuptools import find_packages, setup

package_name = 'dock_pose_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/dock_perception.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/dock_pose_bridge.yaml',
            'config/camera_apriltag.yaml',
            'config/docking_server.yaml',
            'config/dock_database.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='AprilTag 的 TF 轉成 opennav_docking 用的 detected_dock_pose (PoseStamped)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # camera_optical_frame → tag36h11:<id> 的 TF → detected_dock_pose
            'dock_pose_bridge_node = '
            'dock_pose_bridge.dock_pose_bridge_node:main',
        ],
    },
)
