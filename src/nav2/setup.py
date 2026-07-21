from setuptools import find_packages, setup

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/slam_toolbox_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/mapping.launch.py',
            'launch/autonomous_navigation.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='laing0929@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_commander = nav2.nav2_commander:main',
            'keepout_regen = nav2.keepout:main',
        ],
    },
)
