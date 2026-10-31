from setuptools import setup

package_name = 'cooptrans_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinwoolee',
    description='Quasi-static kinematic plant for software-in-the-loop testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'kinematic_plant = cooptrans_sim.kinematic_plant:main',
        ],
    },
)