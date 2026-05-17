from glob import glob
from setuptools import setup, find_packages

package_name = 'joystick_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Colabella',
    maintainer_email='davidecola16@gmail.com',
    description='Joystick to CommandVelocity node for PoliTOcean Nereo ROV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_cmdvel    = joystick_pkg.joy_to_cmdvel:main',
            'rov_cmd_monitor  = joystick_pkg.rov_cmd_monitor:main',
        ],
    },
)
