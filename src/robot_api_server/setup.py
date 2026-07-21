from setuptools import find_packages, setup

package_name = 'robot_api_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/api_server.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
        'pydantic',
        'websockets',
        'PyYAML',
        'Pillow',
    ],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='Winstec Robot API v1.1 server (REST 5000 + WebSocket 5001).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_server = robot_api_server.main:main',
        ],
    },
)
