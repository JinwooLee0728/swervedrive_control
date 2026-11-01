import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'keyboard_to_swerve'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yyj',
    maintainer_email='dbys34@gmail.com',
    description='Keyboard teleop: single swerve robot, or the whole payload.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = keyboard_to_swerve.publisher_member_function:main',
            'swerve_keyboard = keyboard_to_swerve.keyboard_control_swerve_test:main',
            'payload_keyboard = keyboard_to_swerve.payload_keyboard:main',
        ],
    },
)
