from setuptools import find_packages, setup
from glob import glob

package_name = 'gui_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images', glob('gui_pkg/images/*')),
    ],
    install_requires=[
        'setuptools',
        'std_msg',
        'sensor_msgs'
        ],
    zip_safe=True,
    maintainer='Salvatore Lo Sardo',
    maintainer_email='losardosalvatorejr@gmail.com',
    description='GUI for PoliTOcean Nereo ROV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = gui_pkg.gui_node:main',
            'imu_random_node = gui_pkg.simple_publisher_imu_node:main'
        ],
    },
)
