from setuptools import setup

setup(
    name='teleop_twist_keyboard',
    version='0.0.0',
    packages=[],
    py_modules=['teleop_twist_keyboard'],
    install_requires=['setuptools'],
    author='Rohan Agrawal',
    author_email='rohan@osrfoundation.org',
    maintainer='Rohan Agrawal',
    maintainer_email='rohan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Generic keyboard teleop for ROS.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = teleop_twist_keyboard:main',
        ],
    },
)
