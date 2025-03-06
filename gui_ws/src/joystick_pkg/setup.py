from setuptools import setup, find_packages

package_name = 'joystick_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salvatore Lo Sardo',
    maintainer_email='losardosalvatorejr@gmail.com',
    description='Joystick to cmd_vel package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_publisher = joystick_pkg.joy_publisher:main',
            'joy_to_cmdvel = joystick_pkg.joy_to_cmdvel:main'
        ],
    },
)
